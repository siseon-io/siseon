package siseon.backend.dto;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.Map;

@Data
@NoArgsConstructor
@AllArgsConstructor
public class RawPostureRequest {
    private Long profileId;
    private Map<String, Object> userCoord;
}
