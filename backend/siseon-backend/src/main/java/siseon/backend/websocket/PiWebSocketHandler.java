package siseon.backend.websocket;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.*;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

@Component
@RequiredArgsConstructor
public class PiWebSocketHandler extends TextWebSocketHandler {

    private final ObjectMapper mapper;
    private final Set<WebSocketSession> sessions = ConcurrentHashMap.newKeySet();

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        sessions.add(session);
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        sessions.remove(session);
    }

    /**
     * 전달받은 DTO(Object)를 JSON으로 직렬화해
     * 연결된 Pi 세션들에 모두 푸시한다.
     */
    public void sendToAllPi(Object dto) {
        try {
            String json = mapper.writeValueAsString(dto);
            TextMessage tm = new TextMessage(json);
            for (WebSocketSession sess : sessions) {
                if (sess.isOpen()) {
                    sess.sendMessage(tm);
                }
            }
        } catch (Exception e) {
            throw new RuntimeException("Pi WebSocket 전송 오류", e);
        }
    }
}