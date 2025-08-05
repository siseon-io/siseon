package siseon.backend.scheduler;

import lombok.RequiredArgsConstructor;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.launch.JobLauncher;
import org.springframework.batch.core.JobParametersBuilder;
import org.springframework.stereotype.Component;
import org.springframework.scheduling.annotation.Scheduled;
import siseon.backend.repository.main.RawPostureRepository;

import java.time.LocalDateTime;

@RequiredArgsConstructor
@Component
public class PostureJobScheduler {

    private final JobLauncher jobLauncher;
    private final Job postureJob;
    private final RawPostureRepository rawPostureRepository;

    @Scheduled(cron = "0 0/30 * * * ?")
    public void runPostureJob() throws Exception {
        long count = rawPostureRepository.count();
        if (count == 0) {
            System.out.println("[SKIP] raw_postures 테이블에 데이터 없음. Batch Job 생략됨.");
            return;
        }

        LocalDateTime now   = LocalDateTime.now();
        LocalDateTime start = now.minusMinutes(30);

        var params = new JobParametersBuilder()
                .addString("startTime", start.toString())
                .addString("endTime",   now.toString())
                .addLong("run.id", System.currentTimeMillis())
                .toJobParameters();

        jobLauncher.run(postureJob, params);
    }
}