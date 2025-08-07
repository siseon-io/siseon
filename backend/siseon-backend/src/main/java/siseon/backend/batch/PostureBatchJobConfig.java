package siseon.backend.batch;

import lombok.RequiredArgsConstructor;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.Step;
import org.springframework.batch.core.configuration.annotation.EnableBatchProcessing;
import org.springframework.batch.core.job.builder.JobBuilder;
import org.springframework.batch.core.launch.support.RunIdIncrementer;
import org.springframework.batch.core.repository.JobRepository;
import org.springframework.batch.core.step.builder.StepBuilder;
import org.springframework.batch.repeat.RepeatStatus;
import org.springframework.batch.core.step.tasklet.Tasklet;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.transaction.PlatformTransactionManager;
import siseon.backend.domain.main.RawPosture;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.repository.main.RawPostureRepository;
import siseon.backend.repository.batch.PostureStatsRepository;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.*;
import java.util.stream.Collectors;

@Configuration
@EnableBatchProcessing(dataSourceRef = "batchDs", transactionManagerRef = "batchTx")
@RequiredArgsConstructor
public class PostureBatchJobConfig {

    private final JobRepository            jobRepository;
    private final RawPostureRepository     rawRepo;
    private final PostureStatsRepository   statsRepo;
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
            if (raws.isEmpty()) {
                return RepeatStatus.FINISHED;
            }

            // 프로필별 그룹핑
            Map<Long, List<RawPosture>> grouped =
                    raws.stream().collect(Collectors.groupingBy(RawPosture::getProfileId));

            for (var entry : grouped.entrySet()) {
                Long pid = entry.getKey();
                List<RawPosture> list = entry.getValue();

                // monitor_coord 평균
                double avgX = list.stream()
                        .mapToDouble(r -> ((Number) r.getMonitorCoord().get("x")).doubleValue())
                        .average().orElse(0.0);
                double avgY = list.stream()
                        .mapToDouble(r -> ((Number) r.getMonitorCoord().get("y")).doubleValue())
                        .average().orElse(0.0);
                double avgZ = list.stream()
                        .mapToDouble(r -> ((Number) r.getMonitorCoord().get("z")).doubleValue())
                        .average().orElse(0.0);

                // user_coord 평균 (le_pupil, re_pupil + pose_data...)
                List<String> keys = List.of(
                        "le_pupil","re_pupil",
                        "nose","le_ear","re_ear","le_shoulder","re_shoulder",
                        "le_elbow","re_elbow","le_wrist","re_wrist",
                        "le_hip","re_hip","le_knee","re_knee","le_ankle","re_ankle"
                );
                Map<String,double[]> sums = new LinkedHashMap<>();
                keys.forEach(k -> sums.put(k, new double[3]));

                for (RawPosture r : list) {
                    @SuppressWarnings("unchecked")
                    Map<String, Object> user = r.getUserCoord();
                    @SuppressWarnings("unchecked")
                    Map<String, Object> pd = (Map<String, Object>) user.get("pose_data");

                    // pupils
                    for (String k : List.of("le_pupil","re_pupil")) {
                        @SuppressWarnings("unchecked")
                        Map<String, Number> p = (Map<String, Number>) user.get(k);
                        double[] arr = sums.get(k);
                        arr[0] += p.get("x").doubleValue();
                        arr[1] += p.get("y").doubleValue();
                        arr[2] += p.get("z").doubleValue();
                    }
                    // pose_data
                    for (String k : keys.subList(2, keys.size())) {
                        @SuppressWarnings("unchecked")
                        Map<String, Number> p = (Map<String, Number>) pd.get(k);
                        double[] arr = sums.get(k);
                        arr[0] += p.get("x").doubleValue();
                        arr[1] += p.get("y").doubleValue();
                        arr[2] += p.get("z").doubleValue();
                    }
                }

                Map<String,Object> userAvg = new LinkedHashMap<>();
                for (String k : keys) {
                    double[] s = sums.get(k);
                    userAvg.put(k, Map.of(
                            "x", s[0] / list.size(),
                            "y", s[1] / list.size(),
                            "z", s[2] / list.size()
                    ));
                }

                // PostureStats 엔티티 생성
                PostureStats stats = PostureStats.builder()
                        .profileId(pid)
                        .monitorCoord(Map.of("avgx", avgX, "avgy", avgY, "avgz", avgZ))
                        .userCoord(Map.of(
                                "le_pupil", userAvg.get("le_pupil"),
                                "re_pupil", userAvg.get("re_pupil"),
                                "pose_data", keys.subList(2, keys.size())
                                        .stream().collect(Collectors.toMap(k -> k, userAvg::get, (a,b) -> b, LinkedHashMap::new))
                        ))
                        .startAt(start)
                        .endAt(end)
                        .durationSeconds((int) Duration.between(start, end).getSeconds())
                        .slotIndex(start.getMinute())
                        .build();

                statsRepo.save(stats);
            }

            return RepeatStatus.FINISHED;
        };
    }

    @Bean
    public Step cleanupStep() {
        return new StepBuilder("cleanupStep", jobRepository)
                .tasklet((contrib, ctx) -> {
                    LocalDateTime end = LocalDateTime.now();
                    rawRepo.deleteAll(rawRepo.findByCollectedAtLessThanEqual(end));
                    return RepeatStatus.FINISHED;
                }, tx)
                .build();
    }
}