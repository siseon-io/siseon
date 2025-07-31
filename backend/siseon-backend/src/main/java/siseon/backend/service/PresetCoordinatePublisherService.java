package siseon.backend.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.integration.support.MessageBuilder;
import org.springframework.messaging.MessageChannel;
import org.springframework.stereotype.Service;
import siseon.backend.dto.PresetCoordinate;

@Service
@RequiredArgsConstructor
public class PresetCoordinatePublisherService {

    private final ObjectMapper objectMapper;
    private final MessageChannel presetCoordinateOutboundChannel; // MqttConfig에서 정의한 bean

    public void publish(PresetCoordinate coord) throws Exception {
        byte[] json = objectMapper.writeValueAsBytes(coord);
        presetCoordinateOutboundChannel.send(
                MessageBuilder.withPayload(json).build()
        );
    }
}
