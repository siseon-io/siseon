package siseon.backend.dto;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
@Schema(name = "PresetCoordinate", description = "MQTT로 발행되는 좌표 정보 모델")
public class PresetCoordinate {

    @Schema(description = "X 축 위치값", example = "83.23")
    private double x;

    @Schema(description = "Y 축 위치값", example = "72.56")
    private double y;

    @Schema(description = "Z 축 위치값", example = "105.89")
    private double z;
}
