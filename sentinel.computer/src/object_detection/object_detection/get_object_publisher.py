import json
from rclpy.node import Node
from std_msgs.msg import String
from .minio_downloader import MinioDownloader
from .id import UNIQUE_ID
class GetObjectPublisher(Node):
    def __init__(self):
        super().__init__('get_object_publisher')
        self.publisher_ = self.create_publisher(String, 'get_object_links', 10)
        self.minio = MinioDownloader()

        self.bucket_name = f'{UNIQUE_ID}-ros2-bucket'
        self.prefix = 'objects/'  # Adjust as needed


    def publish_links(self):
        try:
            object_keys = self.minio.list_objects(self.bucket_name, self.prefix)
            all_data = []

            for key in object_keys:
                urls = self.minio.list_presigned_urls(self.bucket_name, key)
                for url in urls:
                    filename = key.split("/")[-1]  # Get the actual filename in case it's in a folder
                    if "_" in filename:
                        class_part, id_part = filename.split("_", 1)
                        id_part = id_part.rsplit(".", 1)[0]  # Remove file extension
                        all_data.append({
                            "id": id_part,
                            "class": class_part,
                            "url": url
                        })

            # Convert list of dicts to a JSON string
            json_data = json.dumps(all_data)
            print(json_data)
            msg = String()
            msg.data = json_data
            self.publisher_.publish(msg)

            # self.get_logger().info(f"Published {len(all_data)} URLs as JSON")
        except Exception as e:
            self.get_logger().error(f"Error publishing object links: {str(e)}")

    def publish_one(self, id):
        try:
            object_keys = self.minio.list_objects(self.bucket_name, self.prefix)
            all_data = []

            for key in object_keys:
                urls = self.minio.list_presigned_urls(self.bucket_name, key)
                for url in urls:
                    filename = key.split("/")[-1]  # Get the actual filename in case it's in a folder
                    if "_" in filename:
                        class_part, id_part = filename.split("_", 1)
                        id_part = id_part.rsplit(".", 1)[0]  # Remove file extension

                        if id_part == str(id):  # Filter only the requested id
                            all_data.append({
                                "id": id_part,
                                "class": class_part,
                                "url": url
                            })

            # Convert list of dicts to a JSON string
            json_data = json.dumps(all_data)
            print(json_data)
            msg = String()
            msg.data = json_data
            self.publisher_.publish(msg)

            # self.get_logger().info(f"Published {len(all_data)} URLs for ID={id} as JSON")
        except Exception as e:
            self.get_logger().error(f"Error publishing object links: {str(e)}")
