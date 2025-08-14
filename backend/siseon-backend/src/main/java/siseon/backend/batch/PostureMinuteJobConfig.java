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
            // [start, endExclusive] = 직전 1분 창(상한 배제)
            LocalDateTime end = LocalDateTime.now(KST).truncatedTo(ChronoUnit.MINUTES);
            LocalDateTime start = end.minusMinutes(1);
            LocalDateTime endExclusive = end.minusNanos(1);

            // 'Between'은 양끝 포함이므로 상한을 end-1ns로 전달해 중복 방지
            List<RawPosture> raws = rawRepo.findByCollectedAtBetween(start, endExclusive);
            if (raws.isEmpty()) return RepeatStatus.FINISHED;

            var byProfile = raws.stream().collect(Collectors.groupingBy(RawPosture::getProfileId));

            for (var entry : byProfile.entrySet()) {
                Long pid = entry.getKey();
                List<RawPosture> list = entry.getValue();

                // 1) 분 구간의 모든 posture를 분석해 결과 수집(각도/사유 포함)
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

                // 2) valid 계산
                long validCount = results.stream()
                        .filter(PostureAnalysisUtil.PostureResult::isValidPosture)
                        .count();
                boolean valid = validCount >= Math.ceil(VALID_RATIO * results.size());

                // 3) badReasons 생성 (양호면 템플릿, 아니면 대표 bad 결과 사용)
                Map<String, Object> badReasons = valid
                        ? Map.of("valid", true, "reasons", List.of(), "summary", "양호")
                        : results.stream()
                        .filter(r -> !r.isValidPosture())
                        .findFirst()
                        .map(PostureAnalysisUtil::buildBadReasonsMap)
                        .orElse(Map.of("valid", true, "reasons", List.of(), "summary", "양호"));

                // 4) 모니터 평균 좌표 및 슬롯 인덱스 산출
                double avgX = list.stream().mapToDouble(r -> num(r.getMonitorCoord(), "x")).average().orElse(0.0);
                double avgY = list.stream().mapToDouble(r -> num(r.getMonitorCoord(), "y")).average().orElse(0.0);
                double avgZ = list.stream().mapToDouble(r -> num(r.getMonitorCoord(), "z")).average().orElse(0.0);

                // 하루 기준 분 인덱스(0..1439): 창의 시작 분을 슬롯으로 사용
                int slotIndex = start.getHour() * 60 + start.getMinute();

                // 5) 저장
                PostureStats stats = PostureStats.builder()
                        .profileId(pid)
                        .monitorCoord(Map.of("avgx", avgX, "avgy", avgY, "avgz", avgZ))
                        .userCoord(Map.of("pose_data", Map.of())) // 집계물이므로 비워둠
                        .startAt(start)
                        .endAt(end)
                        .durationSeconds(60)
                        .slotIndex(slotIndex)
                        .validPosture(valid)
                        .build();

                // ✅ badReasons 세팅
                stats.setBadReasons(badReasons);

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

    // Map<String, ?> 로 받아 호출부(Map<String, Number> 또는 Map<String, Object>) 모두 허용
    private static double num(Map<String, ?> m, String key) {
        if (m == null) return 0.0;
        Object v = m.get(key);
        return (v instanceof Number) ? ((Number) v).doubleValue() : 0.0;
    }

    @Bean
    public Step rawCleanupStep() {
        return new StepBuilder("rawCleanupStep", jobRepository)
                .tasklet((contrib, ctx) -> {
                    // 처리 완료된 구간: [*, endExclusive] 까지 정리
                    LocalDateTime end = LocalDateTime.now(KST).truncatedTo(ChronoUnit.MINUTES);
                    LocalDateTime endExclusive = end.minusNanos(1);
                    rawRepo.deleteAll(rawRepo.findByCollectedAtLessThanEqual(endExclusive));
                    return RepeatStatus.FINISHED;
                }, tx)
                .build();
    }
}