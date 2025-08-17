package siseon.backend.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import siseon.backend.dto.ControlMode;

@Service
@RequiredArgsConstructor
public class ControlModeService {

    private final ObjectMapper objectMapper;

    /**
     * /control_mode 메시지 처리 (수동 모드 전환 등)
     * 여기서는 단순히 payload 를 파싱해 로직을 실행하거나 로깅만 합니다.
     */
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
