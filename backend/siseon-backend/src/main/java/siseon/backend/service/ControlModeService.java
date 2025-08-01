package siseon.backend.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import siseon.backend.dto.ControlMode;

@Service
@RequiredArgsConstructor
public class ControlModeService {

    private final ObjectMapper objectMapper;

    public void handleControlMode(byte[] payload) throws Exception {
        ControlMode mode = objectMapper.readValue(payload, ControlMode.class);

        switch (mode.getMode().toUpperCase()) {
            case "MANUAL":
                // 수동 모드 전환 로직
                System.out.println("Manual mode activated for profile " + mode.getProfileId());
                break;
            case "FIX":
                // 필요시 Fix 모드 처리
                break;
            default:
                // 알 수 없는 모드
                throw new IllegalArgumentException("Unknown mode: " + mode.getMode());
        }
    }
}
