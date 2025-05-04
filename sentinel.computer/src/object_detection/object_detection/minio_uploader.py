import io
import boto3
from botocore.client import Config
from rclpy.node import Node

class MinioUploader(Node):
    def __init__(self):
        super().__init__("minio_uploaderS")

        self.s3 = boto3.client(
            's3',
            endpoint_url='http://localhost:9000',
            aws_access_key_id='minioadmin',
            aws_secret_access_key='minioadmin',
            config=Config(signature_version='s3v4'),
            region_name='us-east-1'
        )
        self.get_logger().info("Minio uploader initialized.")

    def create_bucket_if_not_exists(self, bucket_name):
        try:
            buckets = [b['Name'] for b in self.s3.list_buckets()['Buckets']]
            if bucket_name not in buckets:
                self.s3.create_bucket(Bucket=bucket_name)
                # self.get_logger().info(f"Created bucket: {bucket_name}")
        except Exception as e:
            self.get_logger().error(f"Error creating bucket: {str(e)}")

    def upload_bytes(self, data_bytes, bucket_name, object_name, content_type="application/octet-stream"):
        try:
            self.create_bucket_if_not_exists(bucket_name)
            self.s3.upload_fileobj(
                Fileobj=io.BytesIO(data_bytes),
                Bucket=bucket_name,
                Key=object_name,
                ExtraArgs={'ContentType': content_type}
            )
            # self.get_logger().info(f"Uploaded to {bucket_name}/{object_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to upload file: {str(e)}")

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
