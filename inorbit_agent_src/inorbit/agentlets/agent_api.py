# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
# Agentlet module that serves as an Agent API to receive data from
# the robot using local HTTP.
import threading
import time

from util.overrides import overrides

from .agentlet import Agentlet

PLAIN_TEXT_KEY_VALUES_URL = "/api/v1/data/<key>"
JSON_DATA_URL = "/api/v1/data"

# HTTP request status codes extracted from
# https://www.flaskapi.org/api-guide/status-codes/
# Note: these are defined here since the status module is not
# part of the flask core package.
STATUS_HTTP_200_OK = "200"
STATUS_HTTP_409_CONFLICT = "409"
STATUS_HTTP_415_UNSUPPORTED_MEDIA_TYPE = "415"
STATUS_HTTP_501_NOT_IMPLEMENTED = "501"

DEFAULT_WEBSERVER_PORT = 5000

# Timeout to retry loading the server [seconds]
RETRY_TIMEOUT = 30.0

# Max number of attempts to load server
MAX_LOAD_ATTEMPTS = 4


class ServerThread(threading.Thread):
    """
    Server class. Starts a WSGI application on localhost, on the given
    port. Runs on its own thread.
    """

    def __init__(self, port, app, logger):
        threading.Thread.__init__(self)
        self.srv = make_server("localhost", port, app)
        self.port = port
        self.logger = logger
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        self.logger.info(f"Starting server at Port {self.port}")
        self.srv.serve_forever()

    def shutdown(self):
        self.logger.info(f"Shutting down server at Port {self.port}")
        self.srv.shutdown()


class APIAgentlet(Agentlet):
    """
    Agent API class. Gets post requests from the robot, processes the received data
    and redirects it to the corresponding agentlets.
    """

    def __init__(self, uplink, custom_data_agentlet):
        super(APIAgentlet, self).__init__(uplink)
        # NOTE(adamantivm) Flask app is created during load
        self._app = None
        self._custom_data_agentlet = custom_data_agentlet

        self._server = None
        self._states["api_webserver_port"] = DEFAULT_WEBSERVER_PORT

        # Counter for attempts to load server
        self._server_load_attempt = 0

    @overrides(Agentlet)
    def load(self, runlevel):
        # Dynamically load Flask dependencies, to be more protected from possible import errors
        # TODO(adamantivm) Generalize this as a 'try to load dependency' kind of thing
        try:
            from flask import Flask
            from flask import request
            from flask import Response
            from werkzeug.serving import make_server  # this is installed as a flask dependency

            global Flask
            global request
            global Response
            global make_server
        except Exception as e:
            self.once_logger.exception("Flask load", "Exception importing Flask.")
            return False

        if self._app is None:
            self._app = Flask(__name__)

        self._states["runlevel"] = runlevel
        # Add an URL rule for each of the URLS handled by the API.
        self._app.add_url_rule(
            PLAIN_TEXT_KEY_VALUES_URL, "key_values", self._on_key_values, methods=["POST"]
        )
        self._app.add_url_rule(JSON_DATA_URL, "json_data", self._on_json_data, methods=["POST"])

        # Launch server starter thread to make the webserver run on the
        # configured port. If there is no configuration available, use the
        # default port number.
        threading.Thread(target=self._start_server_loop).start()

        self._states["loaded"] = True

        # Send a state update to the cloud
        self.publish_state(self.uplink, self._states)
        return True

    @overrides(Agentlet)
    def unload(self):
        # Shutdown server and kill thread
        try:
            self._shutdown_server()
        except Exception as e:
            self.logger.exception("Unable to shutdown webserver")
        self._server_loader_running = False
        self._states["loaded"] = False
        return True

    @overrides(Agentlet)
    def set_runlevel(self, runlevel):
        self._states["runlevel"] = runlevel

    @overrides(Agentlet)
    def set_state(self, state):
        if "api_webserver_port" in state.keys():
            if self._states["loaded"]:
                # Only restart the server if port number is updated
                if state["api_webserver_port"] != self._states["api_webserver_port"]:
                    try:
                        # Restart server with the new config
                        self._shutdown_server()
                        self._start_server(state["api_webserver_port"])
                    except Exception as e:
                        self.logger.exception(
                            f"Unable to restart agent API on port {state['api_webserver_port']}"
                        )
            self._states["api_webserver_port"] = state["api_webserver_port"]

        # Send a state update
        self.publish_state(self.uplink, self._states)

    def _start_server_loop(self):
        """
        Server starter loop. It runs on a separate thread and tries to start the
        server thread when the agentlet is loaded. In case of error, it will try
        again after RETRY_TIMEOUT seconds, until MAX_LOAD_ATTEMPTS is reached.
        """

        self._server_loader_running = True

        while self._server_loader_running and self._server_load_attempt < MAX_LOAD_ATTEMPTS:
            try:
                self._start_server(self._states["api_webserver_port"])
                self._server_loader_running = False
            except Exception as e:
                self._server_load_attempt += 1
                self.logger.warning(f"Unable to load server. Attempt {self._server_load_attempt}.")
                time.sleep(RETRY_TIMEOUT)
        self.logger.info("Server starter thread shutting down.")

    def _start_server(self, port):
        self._server = ServerThread(port, self._app, self.logger)
        self._server.daemon = True
        self._server.start()

    def _shutdown_server(self):
        self._server.shutdown()

    def _on_key_values(self, key):
        """
        Callback for key-values POST requests made on PLAIN_TEXT_KEY_VALUES_URL.
        Takes the key and values received and redirects them to the
        CustomDataAgentlet.
        """

        # Sanity check. Only accept data of text/plain type.
        if request.headers["Content-Type"] != "text/plain":
            return Response(
                "Expected content-type: text/plain", STATUS_HTTP_415_UNSUPPORTED_MEDIA_TYPE
            )

        try:
            data = request.get_data()
            self._custom_data_agentlet.process_key_value_pair_from_api(key, data)
            return Response("OK", STATUS_HTTP_200_OK)
        except Exception as e:
            return Response("Error processing request", STATUS_HTTP_409_CONFLICT)

    def _on_json_data():
        """
        Callback for POST requests made on JSON_DATA_URL.
        Note (Flor_Grosso): This URL is not yet supported. The method is left here
        to be completed when needed.
        Consider enabling this as a way to receive multiple key value pairs per
        request or other types of data.
        """

        if request.headers["Content-Type"] != "application/json":
            Response(
                "Expected content-type: application/json", STATUS_HTTP_415_UNSUPPORTED_MEDIA_TYPE
            )

        try:
            data = request.get_data()
            # TODO (Flor_Grosso): Implement data processing.
            return Response("Not yet implemented", STATUS_HTTP_501_NOT_IMPLEMENTED)

        except Exception as e:
            return Response("Error processing request", STATUS_HTTP_409_CONFLICT)
