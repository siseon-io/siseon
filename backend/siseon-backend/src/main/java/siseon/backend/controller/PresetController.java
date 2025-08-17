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

    // 수동 프리셋 저장
    @PostMapping
    public ResponseEntity<PresetResponse> createPreset(
            @RequestBody @Valid PresetRequest request
    ) {
        return ResponseEntity.ok(presetService.createFromRaw(request));
    }

    // 자동(확정) 프리셋 저장
    @PostMapping("/confirm")
    public ResponseEntity<PresetResponse> confirmPreset(
            @RequestBody @Valid PresetRequest request
    ) {
        return ResponseEntity.ok(presetService.createFromStats(request));
    }

    // 프로필별 프리셋 목록
    @GetMapping("/profile/{profileId}")
    public ResponseEntity<List<PresetResponse>> getPresetsByProfile(
            @PathVariable Long profileId
    ) {
        return ResponseEntity.ok(presetService.getPresetsByProfile(profileId));
    }

    // 프리셋 수정
    @PutMapping("/{presetId}")
    public ResponseEntity<PresetResponse> updatePreset(
            @PathVariable Long presetId,
            @RequestBody @Valid PresetRequest request
    ) {
        return ResponseEntity.ok(presetService.updatePreset(presetId, request));
    }

    // 프리셋 삭제
    @DeleteMapping("/{presetId}")
    public ResponseEntity<Void> deletePreset(@PathVariable Long presetId) {
        presetService.deletePreset(presetId);
        return ResponseEntity.noContent().build();
    }
}