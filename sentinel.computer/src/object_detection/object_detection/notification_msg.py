import uuid
from dataclasses import dataclass
from datetime import datetime

@dataclass
class NotificationMsg:    
    id: str
    data: str
    timestamp: str
    type: str
    
    @staticmethod
    def create(data, msg_type="INFO"):
        return NotificationMsg(
            id=str(uuid.uuid4()),
            data=data,
            timestamp=datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ"),
            type=msg_type
        )