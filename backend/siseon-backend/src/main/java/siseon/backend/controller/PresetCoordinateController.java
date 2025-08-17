package siseon.backend.controller;

import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.responses.ApiResponse;
import io.swagger.v3.oas.annotations.media.Content;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.PresetCoordinate;
import siseon.backend.dto.PresetPublishRequest;
import siseon.backend.service.PresetCoordinatePublisherService;
import siseon.backend.service.PresetService;

@RestController
@RequestMapping("/api/preset-coordinate")
@RequiredArgsConstructor
public class PresetCoordinateController {

    private final PresetService presetService;
    private final PresetCoordinatePublisherService publisher;

    @Operation(
            summary = "프리셋 좌표 발행",
            responses = {
                    @ApiResponse(
                            responseCode = "200",
                            description = "발행된 좌표 정보",
                            content = @Content(schema = @Schema(implementation = PresetCoordinate.class))
                    )
            }
    )
    @PostMapping
    public ResponseEntity<PresetCoordinate> publishCoordinate(
            @RequestBody PresetPublishRequest req
    ) throws Exception {
        PresetCoordinate coord = presetService.getPresetCoordinate(
                req.getProfileId(),
                req.getPresetId()
        );

        String serialNumber = presetService.getSerialNumberByProfileId(req.getProfileId());
        publisher.publish(coord, serialNumber);

        return ResponseEntity.ok(coord);
    }
}
