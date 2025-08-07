package siseon.backend.controller;

import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.PresetRequest;
import siseon.backend.dto.PresetResponse;
import siseon.backend.service.PresetService;


import java.util.List;

@RestController
@RequestMapping("/api/preset")
@RequiredArgsConstructor
public class PresetController {

    private final PresetService presetService;

    // manual
    @PostMapping
    public ResponseEntity<PresetResponse> createPreset(
            @RequestBody @Valid PresetRequest request
    ) {
        return ResponseEntity.ok(presetService.createFromRaw(request));
    }

    // confirm
    @PostMapping("/confirm")
    public ResponseEntity<PresetResponse> confirmPreset(
            @RequestBody @Valid PresetRequest request
    ) {
        return ResponseEntity.ok(presetService.createFromStats(request));
    }

    @GetMapping("/profile/{profileId}")
    public ResponseEntity<List<PresetResponse>> getPresetsByProfile(
            @PathVariable Long profileId
    ) {
        return ResponseEntity.ok(presetService.getPresetsByProfile(profileId));
    }

    @PutMapping("/{presetId}")
    public ResponseEntity<PresetResponse> updatePreset(
            @PathVariable Long presetId,
            @RequestBody @Valid PresetRequest request
    ) {
        return ResponseEntity.ok(presetService.updatePreset(presetId, request));
    }

    @DeleteMapping("/{presetId}")
    public ResponseEntity<Void> deletePreset(@PathVariable Long presetId) {
        presetService.deletePreset(presetId);
        return ResponseEntity.noContent().build();
    }
}
