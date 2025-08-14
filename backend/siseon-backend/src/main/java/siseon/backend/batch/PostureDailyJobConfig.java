package siseon.backend.batch;

import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.Step;
import org.springframework.batch.core.job.builder.JobBuilder;
import org.springframework.batch.core.launch.support.RunIdIncrementer;
import org.springframework.batch.core.repository.JobRepository;
import org.springframework.batch.core.step.builder.StepBuilder;
import org.springframework.batch.core.step.tasklet.Tasklet;
import org.springframework.batch.repeat.RepeatStatus;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.transaction.PlatformTransactionManager;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.batch.PostureStatsDay;
import siseon.backend.repository.batch.PostureStatsDayRepository;
import siseon.backend.repository.batch.PostureStatsRepository;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Slf4j
@Configuration
@RequiredArgsConstructor
public class PostureDailyJobConfig {

    private static final ZoneId KST = ZoneId.of("Asia/Seoul");

    private final JobRepository jobRepository;
    private final PlatformTransactionManager tx;
    private final PostureStatsRepository statsRepo;
    private final PostureStatsDayRepository dayRepo;

    @Bean
    public Job postureDailyJob(Step dailyAggregateStep, Step dailyCleanupStep) {
        return new JobBuilder("postureDailyJob", jobRepository)
                .incrementer(new RunIdIncrementer())
                .start(dailyAggregateStep)
                .next(dailyCleanupStep)
                .build();
    }

    @Bean
    public Step dailyAggregateStep() {
        return new StepBuilder("dailyAggregateStep", jobRepository)
                .tasklet(dailyAggregateTasklet(), tx)
                .build();
    }

    @Bean
    public Step dailyCleanupStep() {
        return new StepBuilder("dailyCleanupStep", jobRepository)
                .tasklet(dailyCleanupTasklet(), tx)
                .build();
    }

    @Bean
    public Tasklet dailyAggregateTasklet() {
        return (contrib, ctx) -> {
            LocalDate statDate = resolveStatDate(ctx.getStepContext().getJobParameters());
            LocalDateTime start = statDate.atStartOfDay();
            LocalDateTime end   = statDate.plusDays(1).atStartOfDay();

            List<PostureStats> stats = statsRepo.findAllByStartAtInDay(start, end);
            if (stats.isEmpty()) {
                log.info("[dailyAggregate] none: {}, [{}, {})", statDate, start, end);
                return RepeatStatus.FINISHED;
            }

            var agg = stats.stream().collect(Collectors.groupingBy(
                    PostureStats::getProfileId,
                    Collectors.collectingAndThen(Collectors.toList(), list -> {
                        long good = list.stream().filter(s -> Boolean.TRUE.equals(s.getValidPosture())).count();
                        long bad  = list.size() - good;
                        return new long[]{good, bad};
                    })
            ));

            int inserted = 0;
            for (var e : agg.entrySet()) {
                Long pid = e.getKey();
                long[] gb = e.getValue();
                if (dayRepo.existsByProfileIdAndStatDate(pid, statDate)) continue;

                var row = PostureStatsDay.builder()
                        .profileId(pid)
                        .statDate(statDate)
                        .goodCount((int) gb[0])
                        .badCount((int) gb[1])
                        .createdAt(LocalDateTime.now(KST))
                        .build();
                dayRepo.save(row);
                inserted++;
            }

            log.info("[dailyAggregate] minutes={}, profiles={}, inserted={}, date={}, [{}, {})",
                    stats.size(), agg.size(), inserted, statDate, start, end);
            return RepeatStatus.FINISHED;
        };
    }

    @Bean
    public Tasklet dailyCleanupTasklet() {
        return (contrib, ctx) -> {
            LocalDate statDate = resolveStatDate(ctx.getStepContext().getJobParameters());
            LocalDateTime start = statDate.atStartOfDay();
            LocalDateTime end   = statDate.plusDays(1).atStartOfDay();

            int deleted = statsRepo.deleteAllByStartAtInDay(start, end);
            log.info("[dailyCleanup] deleted={}, date={}, [{}, {})", deleted, statDate, start, end);
            return RepeatStatus.FINISHED;
        };
    }

    // 'statDate' (yyyy-MM-dd) 없으면 전일(KST)
    private LocalDate resolveStatDate(Map<String, Object> paramsMap) {
        if (paramsMap != null) {
            Object v = paramsMap.get("statDate");
            if (v != null) {
                try { return LocalDate.parse(v.toString()); }
                catch (Exception ignored) {}
            }
        }
        return LocalDate.now(KST).minusDays(1);
    }
}