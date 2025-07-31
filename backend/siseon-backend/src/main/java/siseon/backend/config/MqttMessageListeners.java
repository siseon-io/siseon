package siseon.backend.config;

import lombok.RequiredArgsConstructor;
import org.springframework.integration.annotation.ServiceActivator;
import org.springframework.messaging.Message;
import org.springframework.stereotype.Component;
import siseon.backend.service.ControlModeService;
import siseon.backend.service.PairingService;

@Component
@RequiredArgsConstructor
public class MqttMessageListeners {

    private final ControlModeService controlModeService;
    private final PairingService pairingService;

    @ServiceActivator(inputChannel = "controlModeChannel")
    public void onControlMode(Message<byte[]> msg) throws Exception {
        controlModeService.handleControlMode(msg.getPayload());
    }

    @ServiceActivator(inputChannel = "requestPairChannel")
    public void onRequestPair(Message<byte[]> msg) throws Exception {
        pairingService.handleRequestPair(msg.getPayload());
    }
}
