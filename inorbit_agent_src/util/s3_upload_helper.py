# Copyright (c) 2018, InOrbit, Inc.
# All rights reserved.
import os

import boto3


class S3_Client(object):
    """
    Interacts with amazon's S3.
    """

    def __init__(self, config):
        """
        Constructs a S3 helper to upload files.

        The config object contains the accessKey, secretKey, company and bucket properties
        telling where files will be uploaded and under which identity.
        """

        self.session = boto3.Session(
            aws_access_key_id=config["accessKey"], aws_secret_access_key=config["secretKey"]
        )
        self.s3 = self.session.resource("s3")
        self.bucket_name = config["bucket"]
        self.bucket = self.s3.Bucket(self.bucket_name)
        self.company = config["company"]

    def upload_file(self, filename, location=None, callback=None):
        """
        Uploads a file to the class bucket to the specified location.
        filename: The file location string.
        location: A string defining where the file should be stored, if left
        undefined it uses the filename as its filename and the company id as folder.
        Using amazon s3 standards you can store them in folders using slashes
        in its name 'company/robot/test.py'
        """

        optionalLocation = "" if location is None else (location + "/")
        key = self.company + "/" + optionalLocation + os.path.basename(filename)
        # Uploads the given file using a managed uploader, which will split up
        # large files automatically and upload parts in parallel.
        self.bucket.upload_file(filename, key, Callback=callback)
        # Return the object URL. Note this may be a protected object (ie. needs credentials to
        # access it)
        return f"https://s3.amazonaws.com/{self.bucket_name}/{key}"
