package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.format.annotation.DateTimeFormat.ISO;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import siseon.backend.dto.PostureStatsResponse;
import siseon.backend.dto.PostureStatsDayResponse;
import siseon.backend.service.PostureStatsService;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;

@RestController
@RequestMapping("/api/posture-stats")
@RequiredArgsConstructor
public class PostureStatsController {

    private final PostureStatsService postureStatsService;

    @GetMapping
    public ResponseEntity<List<? extends PostureStatsResponse>> getStats(
            @RequestParam Long profileId,
            @RequestParam String period,
            @RequestParam(required = false) @DateTimeFormat(iso = ISO.DATE_TIME) LocalDateTime from,
            @RequestParam(required = false) @DateTimeFormat(iso = ISO.DATE_TIME) LocalDateTime to
    ) {
        String p = period.toLowerCase();

        if ("day".equals(p)) {
            // day 조회: from/to가 LocalDateTime으로 들어오면 날짜로 변환
            LocalDate fromDate = (from != null) ? from.toLocalDate() : null;
            LocalDate toDate   = (to   != null) ? to.toLocalDate()   : null;

            List<PostureStatsDayResponse> rows =
                    postureStatsService.getDailyStats(profileId, fromDate, toDate);
            return ResponseEntity.ok(rows);
        }

        // 기본: minute 조회 (기존 로직)
        List<PostureStatsResponse> rows =
                postureStatsService.getStats(profileId, p, from, to);
        return ResponseEntity.ok(rows);
    }
}