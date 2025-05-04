import io
import boto3
from botocore.client import Config
from rclpy.node import Node

class MinioDownloader(Node):
    def __init__(self):
        super().__init__("minio_downloader")

        self.s3 = boto3.client(
            's3',
            endpoint_url='http://localhost:9000',
            aws_access_key_id='minioadmin',
            aws_secret_access_key='minioadmin',
            config=Config(signature_version='s3v4'),
            region_name='us-east-1'
        )
        self.get_logger().info("MinIO downloader initialized.")

    def list_objects(self, bucket_name, prefix=''):
        try:
            paginator = self.s3.get_paginator('list_objects_v2')
            page_iterator = paginator.paginate(Bucket=bucket_name, Prefix=prefix)

            keys = []
            for page in page_iterator:
                contents = page.get('Contents', [])
                keys.extend([obj['Key'] for obj in contents])

            # self.get_logger().info(f"Found {len(keys)} objects in {bucket_name}/{prefix}")
            return keys
        except Exception as e:
            self.get_logger().error(f"Failed to list objects: {str(e)}")
            return []

    def get_presigned_url(self, bucket_name, object_key, expiration=3600):
        try:
            url = self.s3.generate_presigned_url(
                'get_object',
                Params={'Bucket': bucket_name, 'Key': object_key},
                ExpiresIn=expiration
            )
            # self.get_logger().info(f"Generated URL for {object_key}")
            return url
        except Exception as e:
            self.get_logger().error(f"Failed to generate presigned URL: {str(e)}")
            return None

    def list_presigned_urls(self, bucket_name, prefix='objects/', expiration=3600):
        try:
            keys = self.list_objects(bucket_name, prefix)
            urls = []
            for key in keys:
                url = self.get_presigned_url(bucket_name, key, expiration)
                if url:
                    urls.append(url)
            # self.get_logger().info(f"Generated {len(urls)} presigned URLs under {bucket_name}/{prefix}")
            return urls
        except Exception as e:
            self.get_logger().error(f"Error listing presigned URLs: {str(e)}")
            return []
