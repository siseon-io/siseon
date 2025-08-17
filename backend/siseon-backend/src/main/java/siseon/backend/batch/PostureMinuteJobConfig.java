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
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.main.RawPosture;
import siseon.backend.repository.batch.PostureStatsRepository;
import siseon.backend.repository.main.RawPostureRepository;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.temporal.ChronoUnit;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

@Configuration
@EnableBatchProcessing(dataSourceRef = "batchDs", transactionManagerRef = "batchTx")
@RequiredArgsConstructor
public class PostureMinuteJobConfig {

    private static final double VALID_RATIO = 0.8; // 80% 이상이면 정상
    private static final ZoneId KST = ZoneId.of("Asia/Seoul");

    private final JobRepository jobRepository;
    private final PlatformTransactionManager tx;
    private final RawPostureRepository rawRepo;
    private final PostureStatsRepository statsRepo;

    @Bean
    public Job postureMinuteJob(Step minuteProcessStep, Step rawCleanupStep) {
        return new JobBuilder("postureMinuteJob", jobRepository)
                .incrementer(new RunIdIncrementer())
                .start(minuteProcessStep)
                .next(rawCleanupStep)
                .build();
    }

    @Bean
    public Step minuteProcessStep() {
        return new StepBuilder("minuteProcessStep", jobRepository)
                .tasklet(minuteTasklet(), tx)
                .build();
    }

    @Bean
    public Tasklet minuteTasklet() {
        return (contrib, ctx) -> {
            LocalDateTime end = LocalDateTime.now(KST).truncatedTo(ChronoUnit.MINUTES);
            LocalDateTime start = end.minusMinutes(1);
            LocalDateTime endExclusive = end.minusNanos(1);

            List<RawPosture> raws = rawRepo.findByCollectedAtBetween(start, endExclusive);
            if (raws.isEmpty()) return RepeatStatus.FINISHED;

            var byProfile = raws.stream().collect(Collectors.groupingBy(RawPosture::getProfileId));

            for (var entry : byProfile.entrySet()) {
                Long pid = entry.getKey();
                List<RawPosture> list = entry.getValue();

                var results = list.stream()
                        .map(r -> {
                            @SuppressWarnings("unchecked")
                            Map<String, Map<String, Number>> pd =
                                    r.getUserCoord() == null ? null :
                                            (Map<String, Map<String, Number>>) r.getUserCoord().get("pose_data");

                            var joints = Map.<String, double[]>of(
                                    "nose", toArr(pd, "nose"),
                                    "le_ear", toArr(pd, "le_ear"),
                                    "re_ear", toArr(pd, "re_ear"),
                                    "le_shoulder", toArr(pd, "le_shoulder"),
                                    "re_shoulder", toArr(pd, "re_shoulder"),
                                    "le_hip", toArr(pd, "le_hip"),
                                    "re_hip", toArr(pd, "re_hip")
                            );
                            return PostureAnalysisUtil.analyze(joints);
                        })
                        .toList();

                long validCount = results.stream()
                        .filter(PostureAnalysisUtil.PostureResult::isValidPosture)
                        .count();
                boolean valid = validCount >= Math.ceil(VALID_RATIO * results.size());

                Map<String, Object> badReasons = valid
                        ? Map.of("valid", true, "reasons", List.of(), "summary", "양호")
                        : results.stream()
                        .filter(r -> !r.isValidPosture())
                        .findFirst()
                        .map(PostureAnalysisUtil::buildBadReasonsMap)
                        .orElse(Map.of("valid", true, "reasons", List.of(), "summary", "양호"));

                // le_eye / re_eye 평균 계산
                double avgLeX = list.stream().mapToDouble(r -> numPose(r.getUserCoord(), "le_eye", "x")).average().orElse(0.0);
                double avgLeY = list.stream().mapToDouble(r -> numPose(r.getUserCoord(), "le_eye", "y")).average().orElse(0.0);
                double avgLeZ = list.stream().mapToDouble(r -> numPose(r.getUserCoord(), "le_eye", "z")).average().orElse(0.0);

                double avgReX = list.stream().mapToDouble(r -> numPose(r.getUserCoord(), "re_eye", "x")).average().orElse(0.0);
                double avgReY = list.stream().mapToDouble(r -> numPose(r.getUserCoord(), "re_eye", "y")).average().orElse(0.0);
                double avgReZ = list.stream().mapToDouble(r -> numPose(r.getUserCoord(), "re_eye", "z")).average().orElse(0.0);

                int slotIndex = start.getHour() * 60 + start.getMinute();

                PostureStats stats = PostureStats.builder()
                        .profileId(pid)
                        .lefteyeX(avgLeX)
                        .lefteyeY(avgLeY)
                        .lefteyeZ(avgLeZ)
                        .righteyeX(avgReX)
                        .righteyeY(avgReY)
                        .righteyeZ(avgReZ)
                        .userCoord(Map.of()) // 집계물이므로 비워둠
                        .startAt(start)
                        .endAt(end)
                        .durationSeconds(60)
                        .slotIndex(slotIndex)
                        .validPosture(valid)
                        .badReasons(badReasons)
                        .build();

                statsRepo.save(stats);
            }
            return RepeatStatus.FINISHED;
        };
    }

    private static double[] toArr(Map<String, Map<String, Number>> pd, String key) {
        if (pd == null) return new double[]{0, 0, 0};
        Map<String, Number> c = pd.get(key);
        if (c == null) return new double[]{0, 0, 0};
        return new double[]{
                c.getOrDefault("x", 0).doubleValue(),
                c.getOrDefault("y", 0).doubleValue(),
                c.getOrDefault("z", 0).doubleValue()
        };
    }

    private static double numPose(Map<String, Object> userCoord, String joint, String axis) {
        if (userCoord == null) return 0.0;
        Object poseDataObj = userCoord.get("pose_data");
        if (!(poseDataObj instanceof Map)) return 0.0;
        Map<String, Map<String, Number>> poseData = (Map<String, Map<String, Number>>) poseDataObj;
        Map<String, Number> jointMap = poseData.get(joint);
        if (jointMap == null) return 0.0;
        return jointMap.getOrDefault(axis, 0).doubleValue();
    }

    @Bean
    public Step rawCleanupStep() {
        return new StepBuilder("rawCleanupStep", jobRepository)
                .tasklet((contrib, ctx) -> {
                    LocalDateTime end = LocalDateTime.now(KST).truncatedTo(ChronoUnit.MINUTES);
                    LocalDateTime endExclusive = end.minusNanos(1);
                    rawRepo.deleteAll(rawRepo.findByCollectedAtLessThanEqual(endExclusive));
                    return RepeatStatus.FINISHED;
                }, tx)
                .build();
    }
}