package siseon.backend.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import siseon.backend.dto.RequestPair;

@Service
@RequiredArgsConstructor
public class PairingService {

    private final ObjectMapper objectMapper;
    // 필요하면 PairingRepository 같은 것을 주입하세요.

    /**
     * /request_pair 메시지 처리 (IoT 디바이스 페어링 시작)
     */
    public void handleRequestPair(byte[] payload) throws Exception {
        RequestPair req = objectMapper.readValue(payload, RequestPair.class);
        // TODO: DB 저장 또는 외부 페어링 API 호출
        System.out.println("Pairing requested for device " + req.getDeviceMac());
    }
}
