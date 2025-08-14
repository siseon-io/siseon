package siseon.backend.scheduler;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.JobParametersBuilder;
import org.springframework.batch.core.launch.JobLauncher;
import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;
import siseon.backend.repository.main.RawPostureRepository;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;

@Slf4j
@Component
@RequiredArgsConstructor
@EnableScheduling
public class PostureJobScheduler {

    private static final ZoneId KST = ZoneId.of("Asia/Seoul");

    private final JobLauncher jobLauncher;

    // 1분 집계 Job (raw_postures → posture_stats)
    private final Job postureMinuteJob;

    // 자정 일일집계 Job (posture_stats[전일] → posture_stats_day & cleanup)
    private final Job postureDailyJob;

    private final RawPostureRepository rawPostureRepository;

    /**
     * 매 분 0초(KST)마다 실행.
     * - 분 경계로 [start, end) = [end-1분, end] 구간을 집계
     * - 구간 내 raw_postures가 없으면 스킵
     */
    @Scheduled(cron = "0 * * * * *", zone = "Asia/Seoul")
    public void runMinuteJob() {
        // 분 경계 정렬
        LocalDateTime end   = LocalDateTime.now(KST).truncatedTo(ChronoUnit.MINUTES);
        LocalDateTime start = end.minusMinutes(1);

        long countInWindow = rawPostureRepository.countByCollectedAtBetween(start, end);
        if (countInWindow == 0) {
            log.info("[SKIP] 최근 1분(raw_postures) 데이터 없음 ({} ~ {}). Minute Job 생략.", start, end);
            return;
        }

        try {
            var params = new JobParametersBuilder()
                    .addString("startAt", start.toString())
                    .addString("endAt",   end.toString())
                    // 동일 분 재실행 대비 유니크 파라미터
                    .addLong("epochMinute", end.atZone(KST).toEpochSecond())
                    .toJobParameters();

            jobLauncher.run(postureMinuteJob, params);
            log.info("Minute Job 실행 완료: {} ~ {} (raw count={})", start, end, countInWindow);
        } catch (Exception e) {
            log.error("Minute Job 실행 중 오류", e);
        }
    }

    /**
     * 매일 00:05:00(KST) 실행.
     * - "전일 00:00 ~ 당일 00:00" 구간을 일일 집계(Job 내부에서 처리/정리)
     * - 여기서는 트리거만 하고, 집계/정리는 Job 내 tasklet에서 수행
     */
    @Scheduled(cron = "0 5 0 * * *", zone = "Asia/Seoul")
    public void runDailyJob() {
        ZonedDateTime nowKst = ZonedDateTime.now(KST);
        LocalDate today = nowKst.toLocalDate();
        LocalDate statDate = today.minusDays(1);

        try {
            var params = new JobParametersBuilder()
                    .addString("statDate", statDate.toString()) // 전일 날짜 전달
                    .addLong("ts", nowKst.toEpochSecond())      // 유니크 파라미터
                    .toJobParameters();

            jobLauncher.run(postureDailyJob, params);
            log.info("Daily Job 트리거 완료: statDate={}", statDate);
        } catch (Exception e) {
            log.error("Daily Job 실행 중 오류", e);
        }
    }
}