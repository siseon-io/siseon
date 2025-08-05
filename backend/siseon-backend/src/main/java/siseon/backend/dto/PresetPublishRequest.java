package siseon.backend.dto;

import com.fasterxml.jackson.annotation.JsonProperty;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor
public class PresetPublishRequest {
    @JsonProperty("profile_id")
    private Long profileId;

    @JsonProperty("preset_id")
    private Long presetId;
}