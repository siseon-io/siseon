package siseon.backend.batch;

import lombok.RequiredArgsConstructor;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.Step;
import org.springframework.batch.core.configuration.annotation.EnableBatchProcessing;
import org.springframework.batch.core.job.builder.JobBuilder;
import org.springframework.batch.core.launch.support.RunIdIncrementer;
import org.springframework.batch.core.repository.JobRepository;
import org.springframework.batch.core.step.builder.StepBuilder;
import org.springframework.batch.core.step.tasklet.Tasklet;
import org.springframework.batch.repeat.RepeatStatus;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.transaction.PlatformTransactionManager;
import siseon.backend.domain.main.RawPosture;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.repository.main.RawPostureRepository;
import siseon.backend.repository.batch.PostureStatsRepository;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Configuration
@EnableBatchProcessing(dataSourceRef = "batchDs", transactionManagerRef = "batchTx")
@RequiredArgsConstructor
public class PostureBatchJobConfig {

    private static final double VALID_RATIO = 0.8; // 80%% 이상이면 정상으로 판단

    private final JobRepository jobRepository;
    private final RawPostureRepository rawRepo;
    private final PostureStatsRepository statsRepo;
    private final PlatformTransactionManager tx;

    @Bean
    public Job postureJob(Step processStep, Step cleanupStep) {
        return new JobBuilder("postureJob", jobRepository)
                .incrementer(new RunIdIncrementer())
                .start(processStep)
                .next(cleanupStep)
                .build();
    }

    @Bean
    public Step processStep() {
        return new StepBuilder("processStep", jobRepository)
                .tasklet(processTasklet(), tx)
                .build();
    }

    @Bean
    public Tasklet processTasklet() {
        return (contrib, ctx) -> {
            LocalDateTime end   = LocalDateTime.now();
            LocalDateTime start = end.minusMinutes(10);

            List<RawPosture> raws = rawRepo.findByCollectedAtBetween(start, end);
            if (raws.isEmpty()) return RepeatStatus.FINISHED;

            var grouped = raws.stream()
                    .collect(Collectors.groupingBy(RawPosture::getProfileId));

            for (var entry : grouped.entrySet()) {
                Long pid = entry.getKey();
                List<RawPosture> list = entry.getValue();

                long validCount = list.stream()
                        .map(r -> {
                            @SuppressWarnings("unchecked")
                            var pd = (Map<String, Map<String, Number>>) r.getUserCoord().get("pose_data");
                            var joints = Map.of(
                                    "nose",      toArr(pd.get("nose")),
                                    "le_ear",    toArr(pd.get("le_ear")),
                                    "re_ear",    toArr(pd.get("re_ear")),
                                    "le_shoulder", toArr(pd.get("le_shoulder")),
                                    "re_shoulder", toArr(pd.get("re_shoulder")),
                                    "le_hip",    toArr(pd.get("le_hip")),
                                    "re_hip",    toArr(pd.get("re_hip"))
                            );
                            return PostureAnalysisUtil.analyze(joints).isValidPosture();
                        })
                        .filter(Boolean::booleanValue)
                        .count();

                boolean valid = validCount >= Math.ceil(VALID_RATIO * list.size());

                double avgX = list.stream()
                        .mapToDouble(r -> ((Number) r.getMonitorCoord().get("x")).doubleValue())
                        .average().orElse(0.0);
                double avgY = list.stream()
                        .mapToDouble(r -> ((Number) r.getMonitorCoord().get("y")).doubleValue())
                        .average().orElse(0.0);
                double avgZ = list.stream()
                        .mapToDouble(r -> ((Number) r.getMonitorCoord().get("z")).doubleValue())
                        .average().orElse(0.0);

                PostureStats stats = PostureStats.builder()
                        .profileId(pid)
                        .monitorCoord(Map.of("avgx", avgX, "avgy", avgY, "avgz", avgZ))
                        .userCoord(Map.of("pose_data", Map.of()))
                        .startAt(start)
                        .endAt(end)
                        .durationSeconds((int) Duration.between(start, end).getSeconds())
                        .slotIndex(start.getMinute())
                        .validPosture(valid)
                        .build();

                statsRepo.save(stats);
            }
            return RepeatStatus.FINISHED;
        };
    }

    private static double[] toArr(Map<String, Number> c) {
        return new double[]{
                c.get("x").doubleValue(),
                c.get("y").doubleValue(),
                c.get("z").doubleValue()
        };
    }

    @Bean
    public Step cleanupStep() {
        return new StepBuilder("cleanupStep", jobRepository)
                .tasklet((contrib, ctx) -> {
                    LocalDateTime now = LocalDateTime.now();
                    rawRepo.deleteAll(rawRepo.findByCollectedAtLessThanEqual(now));
                    return RepeatStatus.FINISHED;
                }, tx)
                .build();
    }
}