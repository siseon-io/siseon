package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.PostureStatsResponse;
import siseon.backend.service.PostureStatsService;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/posture-stats")
public class PostureStatsController {

    private final PostureStatsService postureStatsService;

    @GetMapping
    public ResponseEntity<List<PostureStatsResponse>> getStats(
            @RequestParam Long profileId,
            @RequestParam String period
    ) {
        List<PostureStatsResponse> stats =
                postureStatsService.getStatsByPeriod(profileId, period.toLowerCase());
        return ResponseEntity.ok(stats);
    }
}
