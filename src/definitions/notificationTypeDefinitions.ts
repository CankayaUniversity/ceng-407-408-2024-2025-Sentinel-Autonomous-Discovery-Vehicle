export type NotificationType = 'INFO' | 'WARNING' | 'ERROR';

export interface NotificationItem {
    id: string;
    data: string;
    timestamp: string;
    type: NotificationType;
}