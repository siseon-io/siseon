// src/main/java/siseon/backend/controller/DeviceController.java
package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.CreateDeviceRequest;
import siseon.backend.dto.DeviceResponse;
import siseon.backend.service.DeviceService;

import java.util.Collections;
import java.util.List;
import java.util.Map;

@RestController
@RequestMapping("/api/device")
@RequiredArgsConstructor
public class DeviceController {
    private final DeviceService deviceService;

    @PostMapping
    public ResponseEntity<DeviceResponse> create(@RequestBody CreateDeviceRequest req) {
        var device = deviceService.register(req.getProfileId(), req.getSerialNumber());
        return ResponseEntity
                .status(201)
                .body(new DeviceResponse(device));
    }

    @GetMapping("/profile/{profileId}")
    public ResponseEntity<Map<String, List<String>>> getSerials(@PathVariable Long profileId) {
        List<String> serials = deviceService.findSerialsByProfile(profileId);
        // "serials" 라는 키로 List<String>을 감싸서 내려줌
        return ResponseEntity.ok(Collections.singletonMap("serials", serials));
    }

    @DeleteMapping("/profile/{profileId}")
    public ResponseEntity<Void> deleteByProfile(@PathVariable Long profileId) {
        deviceService.deleteByProfileId(profileId);
        return ResponseEntity.noContent().build();
    }
}
