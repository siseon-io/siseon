package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.PresetCoordinate;
import siseon.backend.service.PresetCoordinatePublisherService;
import siseon.backend.service.PresetService;

@RestController
@RequestMapping("/api/preset-coordinate")
@RequiredArgsConstructor
public class PresetCoordinateController {

    private final PresetService presetService;
    private final PresetCoordinatePublisherService publisher;

    @PostMapping("/publish/{profileId}")
    public ResponseEntity<Void> publishCoordinate(@PathVariable Long profileId) throws Exception {
        PresetCoordinate coord = presetService.getPresetCoordinate(profileId);
        publisher.publish(coord);
        return ResponseEntity.ok().build();
    }
}
