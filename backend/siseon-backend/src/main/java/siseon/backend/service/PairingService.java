package siseon.backend.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import siseon.backend.dto.RequestPair;

@Service
@RequiredArgsConstructor
public class PairingService {

    private final ObjectMapper objectMapper;

    public void handleRequestPair(byte[] payload) throws Exception {
        RequestPair req = objectMapper.readValue(payload, RequestPair.class);
        System.out.println("Pairing requested for device " + req.getDeviceMac());
    }
}
