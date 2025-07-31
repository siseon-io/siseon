package siseon.backend.config;

import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.integration.channel.DirectChannel;
import org.springframework.integration.mqtt.core.DefaultMqttPahoClientFactory;
import org.springframework.integration.mqtt.core.MqttPahoClientFactory;
import org.springframework.integration.mqtt.inbound.MqttPahoMessageDrivenChannelAdapter;
import org.springframework.integration.mqtt.outbound.MqttPahoMessageHandler;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.MessageHandler;

@Configuration
public class MqttConfig {

    // ────────────────────────────────────────────────────────
    // 설정 값 (application.yml)
    // ────────────────────────────────────────────────────────
    @Value("${spring.mqtt.host}")
    private String brokerUrl;

    @Value("${spring.mqtt.client-id}")
    private String clientId;

    @Value("${spring.mqtt.username}")
    private String username;

    @Value("${spring.mqtt.password}")
    private String password;

    // ────────────────────────────────────────────────────────
    // 토픽 상수
    // ────────────────────────────────────────────────────────
    private static final String TOPIC_CONTROL_MODE    = "control_mode";
    private static final String TOPIC_REQUEST_PAIR    = "request_pair";
    private static final String TOPIC_PRESET_COORDINATE = "preset_coordinate";

    // ────────────────────────────────────────────────────────
    // 1) MQTT 커넥션 팩토리
    // ────────────────────────────────────────────────────────
    @Bean
    public MqttPahoClientFactory mqttClientFactory() {
        DefaultMqttPahoClientFactory factory = new DefaultMqttPahoClientFactory();
        MqttConnectOptions opts = new MqttConnectOptions();
        opts.setServerURIs(new String[]{ brokerUrl });
        opts.setUserName(username);
        opts.setPassword(password.toCharArray());
        factory.setConnectionOptions(opts);
        return factory;
    }

    // ────────────────────────────────────────────────────────
    // 2) Inbound 채널 정의
    // ────────────────────────────────────────────────────────
    @Bean
    public MessageChannel controlModeChannel() {
        return new DirectChannel();
    }

    @Bean
    public MessageChannel requestPairChannel() {
        return new DirectChannel();
    }

    // ────────────────────────────────────────────────────────
    // 3) Inbound 어댑터 등록 (구독)
    // ────────────────────────────────────────────────────────
    @Bean
    public MqttPahoMessageDrivenChannelAdapter controlModeInbound(
            MqttPahoClientFactory factory
    ) {
        MqttPahoMessageDrivenChannelAdapter adapter =
                new MqttPahoMessageDrivenChannelAdapter(
                        clientId + "_sub_control",
                        factory,
                        TOPIC_CONTROL_MODE
                );
        adapter.setOutputChannel(controlModeChannel());
        return adapter;
    }

    @Bean
    public MqttPahoMessageDrivenChannelAdapter requestPairInbound(
            MqttPahoClientFactory factory
    ) {
        MqttPahoMessageDrivenChannelAdapter adapter =
                new MqttPahoMessageDrivenChannelAdapter(
                        clientId + "_sub_pair",
                        factory,
                        TOPIC_REQUEST_PAIR
                );
        adapter.setOutputChannel(requestPairChannel());
        return adapter;
    }

    // ────────────────────────────────────────────────────────
    // 4) Outbound 채널 정의
    // ────────────────────────────────────────────────────────
    @Bean
    public MessageChannel presetCoordinateOutboundChannel() {
        return new DirectChannel();
    }

    // ────────────────────────────────────────────────────────
    // 5) Outbound 어댑터 등록 (발행)
    // ────────────────────────────────────────────────────────
    @Bean
    @ServiceActivator(inputChannel = "presetCoordinateOutboundChannel")
    public MessageHandler presetCoordinateOutboundHandler(
            MqttPahoClientFactory factory
    ) {
        MqttPahoMessageHandler handler =
                new MqttPahoMessageHandler(
                        clientId + "_pub_preset",
                        factory
                );
        handler.setAsync(true);
        handler.setDefaultTopic(TOPIC_PRESET_COORDINATE);
        return handler;
    }
}
