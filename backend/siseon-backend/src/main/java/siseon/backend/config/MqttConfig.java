package siseon.backend.config;

import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;

@Configuration
public class MqttConfig {

    @Value("${spring.mqtt.host}")
    private String brokerUrl;

    @Value("${spring.mqtt.client-id}")
    private String clientId;

    @Value("${spring.mqtt.username}")
    private String username;

    @Value("${spring.mqtt.password}")
    private String password;

    // 기본 토픽 prefix (serialNumber는 발행 시점에 붙임)
    public static final String TOPIC_PRESET_COORDINATE_PREFIX = "/preset_coordinate";

    @Bean
    public MqttPahoClientFactory mqttClientFactory() {
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        factory.setConnectionOptions(mqttConnectOptions());
        return factory;
    }

    private MqttConnectOptions mqttConnectOptions() {
        MqttConnectOptions opts = new MqttConnectOptions();
        opts.setServerURIs(new String[]{brokerUrl});
        opts.setUserName(username);
        opts.setPassword(password.toCharArray());
        return opts;
    }

    @Bean
    public MessageChannel presetCoordinateOutboundChannel() {
        return new DirectChannel();
    }

    @Bean
    @ServiceActivator(inputChannel = "presetCoordinateOutboundChannel")
    public MessageHandler presetCoordinateOutboundHandler(MqttPahoClientFactory factory) {
        MqttPahoMessageHandler handler =
                new MqttPahoMessageHandler(clientId + "_pub_preset", factory);
        handler.setAsync(true);
        // setDefaultTopic 제거 → 발행 시점에 MqttHeaders.TOPIC 설정
        return handler;
    }
}