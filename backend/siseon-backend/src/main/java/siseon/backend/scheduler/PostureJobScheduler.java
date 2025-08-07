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

import java.time.LocalDateTime;

@Slf4j
@Component
@RequiredArgsConstructor
@EnableScheduling
public class PostureJobScheduler {

    private final JobLauncher jobLauncher;
    private final Job postureJob;
    private final RawPostureRepository rawPostureRepository;

    /**
     * 매 10분 정각에 실행
     */
    @Scheduled(cron = "0 0/10 * * * ?")
    public void runPostureJob() {
        LocalDateTime end   = LocalDateTime.now();
        LocalDateTime start = end.minusMinutes(10);

        long countInWindow = rawPostureRepository.countByCollectedAtBetween(start, end);
        if (countInWindow == 0) {
            log.info("[SKIP] 최근 10분(raw_postures) 데이터 없음 ({} ~ {}). Batch Job 생략.", start, end);
            return;
        }

        try {
            var params = new JobParametersBuilder()
                    .addString("startTime", start.toString())
                    .addString("endTime",   end.toString())
                    .addLong("run.id",      System.currentTimeMillis())
                    .toJobParameters();

            jobLauncher.run(postureJob, params);
            log.info("Posture batch job 실행: {} ~ {}", start, end);
        } catch (Exception e) {
            log.error("Posture batch job 실행 중 오류 발생", e);
        }
    }
}