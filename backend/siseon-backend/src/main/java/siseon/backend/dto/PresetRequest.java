package siseon.backend.dto;

import lombok.*;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class PresetRequest {
    private Long profileId;
    private String name;
    private double lefteyeX;
    private double lefteyeY;
    private double lefteyeZ;
    private double righteyeX;
    private double righteyeY;
    private double righteyeZ;
}
