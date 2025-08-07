package siseon.backend.service;

import com.google.api.core.ApiFuture;
import com.google.api.core.ApiFutureCallback;
import com.google.api.core.ApiFutures;
import com.google.common.util.concurrent.MoreExecutors;
import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.Message;
import com.google.firebase.messaging.Notification;
import lombok.extern.slf4j.Slf4j;
import org.springframework.scheduling.annotation.Async;
import org.springframework.stereotype.Service;

import java.util.Map;

@Slf4j
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

        send(message);
    }

    @Async
    public void sendPushAsync(String fcmToken,
                              String title,
                              String body,
                              Map<String,String> data) {
        Message.Builder builder = Message.builder()
                .setToken(fcmToken)
                .setNotification(Notification.builder()
                        .setTitle(title)
                        .setBody(body)
                        .build())
                .putData("type", "posture");

        data.forEach(builder::putData);

        send(builder.build());
    }

    private void send(Message message) {
        ApiFuture<String> future = FirebaseMessaging.getInstance().sendAsync(message);

        ApiFutures.addCallback(future, new ApiFutureCallback<>() {
            @Override
            public void onFailure(Throwable t) {
                log.error("FCM 전송 실패", t);
            }
            @Override
            public void onSuccess(String messageId) {
                log.info("✅ FCM 메시지 전송 성공, messageId={}", messageId);
            }
        }, MoreExecutors.directExecutor());
    }
}
