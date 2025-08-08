package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.format.annotation.DateTimeFormat.ISO;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.PostureStatsResponse;
import siseon.backend.service.PostureStatsService;

import java.time.LocalDateTime;
import java.util.List;

@RestController
@RequestMapping("/api/posture-stats")
@RequiredArgsConstructor
public class PostureStatsController {
    private final PostureStatsService postureStatsService;

    @GetMapping
    public ResponseEntity<List<PostureStatsResponse>> getStats(
            @RequestParam Long profileId,
            @RequestParam String period,
            @RequestParam(required = false)
            @DateTimeFormat(iso = ISO.DATE_TIME) LocalDateTime from,
            @RequestParam(required = false)
            @DateTimeFormat(iso = ISO.DATE_TIME) LocalDateTime to
    ) {
        List<PostureStatsResponse> stats =
                postureStatsService.getStats(profileId, period.toLowerCase(), from, to);
        return ResponseEntity.ok(stats);
    }
}