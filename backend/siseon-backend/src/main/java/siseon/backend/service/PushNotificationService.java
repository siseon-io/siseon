package siseon.backend.service;

import com.google.api.core.ApiFuture;
import com.google.api.core.ApiFutureCallback;
import com.google.api.core.ApiFutures;
import com.google.common.util.concurrent.MoreExecutors;
import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.Message;
import com.google.firebase.messaging.Notification;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;

@Service
public class PushNotificationService {

    @Async
    public void sendPushAsync(String fcmToken, String title, String body) {
        Message message = Message.builder()
                .setToken(fcmToken)
                .setNotification(Notification.builder()
                        .setTitle(title)
                        .setBody(body)
                        .build())
                .putData("type", "posture")
                .build();

        ApiFuture<String> future = FirebaseMessaging.getInstance().sendAsync(message);

        ApiFutures.addCallback(
                future,
                new ApiFutureCallback<>() {
                    @Override
                    public void onFailure(Throwable t) {
                        // TODO: 실패 로깅 또는 재시도 처리
                        System.err.println("❌ FCM 전송 실패: " + t.getMessage());
                    }

                    @Override
                    public void onSuccess(String messageId) {
                        System.out.println("✅ FCM 메시지 ID: " + messageId);
                    }
                },
                MoreExecutors.directExecutor()
        );
    }
}
