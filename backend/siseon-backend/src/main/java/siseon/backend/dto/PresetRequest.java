package siseon.backend.dto;

import jakarta.validation.constraints.NotNull;
import lombok.*;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PresetRequest {

    @NotNull
    private Long profileId;

    @NotNull
    private String name;
}
