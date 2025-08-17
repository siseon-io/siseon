package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.RawPostureRequest;
import siseon.backend.service.RawPostureService;

@RestController
@RequestMapping("/api/raw-postures")
@RequiredArgsConstructor
public class RawPostureController {

    private final RawPostureService rawPostureService;

    @PostMapping
    public ResponseEntity<Void> collect(@RequestBody RawPostureRequest req) {
        rawPostureService.saveRawPosture(req);
        return ResponseEntity.status(HttpStatus.CREATED).build();
    }
}
