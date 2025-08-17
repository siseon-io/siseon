package siseon.backend.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.integration.mqtt.support.MqttHeaders;
import org.springframework.integration.support.MessageBuilder;
import org.springframework.messaging.MessageChannel;
import org.springframework.stereotype.Service;
import siseon.backend.config.MqttConfig;
import siseon.backend.dto.PresetCoordinate;

@Service
@RequiredArgsConstructor
public class PresetCoordinatePublisherService {

    private final ObjectMapper objectMapper;
    private final MessageChannel presetCoordinateOutboundChannel; // MqttConfig에서 정의한 bean

    /**
     * /preset_coordinate/{serialNumber} 토픽으로 좌표 발행
     */
    public void publish(PresetCoordinate coord, String serialNumber) throws Exception {
        byte[] json = objectMapper.writeValueAsBytes(coord);

        String topic = MqttConfig.TOPIC_PRESET_COORDINATE_PREFIX + "/" + serialNumber;

        presetCoordinateOutboundChannel.send(
                MessageBuilder.withPayload(json)
                        .setHeader(MqttHeaders.TOPIC, topic)
                        .build()
        );
    }
}