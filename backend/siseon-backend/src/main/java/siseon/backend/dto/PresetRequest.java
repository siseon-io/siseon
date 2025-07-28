package siseon.backend.dto;

import jakarta.validation.constraints.NotNull;
import lombok.*;

import java.util.Map;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PresetRequest {

    @NotNull
    private Long deviceId;

    @NotNull
    private Long profileId;

    @NotNull
    private String name;

    @NotNull
    private Map<String, Object> position;
}