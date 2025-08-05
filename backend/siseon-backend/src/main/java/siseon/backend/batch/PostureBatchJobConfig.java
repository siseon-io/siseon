package siseon.backend.batch;

import lombok.RequiredArgsConstructor;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.Step;
import org.springframework.batch.core.configuration.annotation.EnableBatchProcessing;
import org.springframework.batch.core.job.builder.JobBuilder;
import org.springframework.batch.core.step.builder.StepBuilder;
import org.springframework.batch.core.repository.JobRepository;
import org.springframework.batch.item.data.RepositoryItemReader;
import org.springframework.batch.item.data.builder.RepositoryItemReaderBuilder;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.batch.core.configuration.annotation.StepScope;
import org.springframework.batch.repeat.RepeatStatus;

import org.springframework.transaction.PlatformTransactionManager;

import siseon.backend.domain.main.RawPosture;
import siseon.backend.domain.batch.PostureStats;
import siseon.backend.domain.batch.PresetSuggestion;
import siseon.backend.listener.RawPostureStepListener;
import siseon.backend.repository.main.RawPostureRepository;
import siseon.backend.repository.batch.PostureStatsRepository;
import siseon.backend.repository.batch.PresetSuggestionRepository;

import java.time.Duration;
import java.time.LocalDateTime;
import java.util.List;

@Configuration
@EnableBatchProcessing(dataSourceRef = "batchDs", transactionManagerRef = "batchTx")
@RequiredArgsConstructor
public class PostureBatchJobConfig {

    private final JobRepository jobRepository;

    private final RawPostureRepository rawPostureRepository;
    private final PostureStatsRepository postureStatsRepository;
    private final PresetSuggestionRepository presetSuggestionRepository;

    private final PlatformTransactionManager batchTx;

    @Bean
    public Job postureJob(Step postureStep, Step cleanupStep) {
        return new JobBuilder("postureJob", jobRepository)
                .start(postureStep)
                .next(cleanupStep)
                .build();
    }

    @Bean
    public Step postureStep(RawPostureStepListener listener) {
        return new StepBuilder("postureStep", jobRepository)
                .<RawPosture, PostureStats>chunk(500, batchTx)
                .reader(reader(null, null))
                .processor(this::toStat)
                .writer(items -> postureStatsRepository.saveAll(items))
                .listener(listener)
                .build();
    }

    @Bean
    public Step cleanupStep() {
        return new StepBuilder("cleanupStep", jobRepository)
                .tasklet((contrib, ctx) -> {
                    var params = ctx.getStepContext()
                            .getStepExecution()
                            .getJobParameters();
                    LocalDateTime start = LocalDateTime.parse(params.getString("startTime"));
                    LocalDateTime end   = LocalDateTime.parse(params.getString("endTime"));
                    rawPostureRepository.deleteAll(
                            rawPostureRepository.findByCollectedAtBetween(start, end)
                    );
                    return RepeatStatus.FINISHED;
                }, batchTx)
                .build();
    }

    @Bean
    @StepScope
    public RepositoryItemReader<RawPosture> reader(
            @org.springframework.beans.factory.annotation.Value("#{jobParameters['startTime']}") String st,
            @org.springframework.beans.factory.annotation.Value("#{jobParameters['endTime']}")   String et) {

        return new RepositoryItemReaderBuilder<RawPosture>()
                .name("rawPostureReader")
                .repository(rawPostureRepository)
                .methodName("findByCollectedAtBetween")
                .arguments(List.of(LocalDateTime.parse(st), LocalDateTime.parse(et)))
                .pageSize(500)
                .sorts(java.util.Collections.singletonMap("id", org.springframework.data.domain.Sort.Direction.ASC))
                .build();
    }

    private PostureStats toStat(RawPosture raw) {
        LocalDateTime end   = raw.getCollectedAt();
        LocalDateTime start = end.minusMinutes(30);
        int duration = (int) Duration.between(start, end).toSeconds();
        int slotIndex = start.getHour() * 2 + start.getMinute() / 30;

        var stat = new PostureStats();
        stat.setProfileId(raw.getProfileId());
        stat.setAvgX(raw.getX()); stat.setAvgY(raw.getY()); stat.setAvgZ(raw.getZ());
        stat.setStartAt(start); stat.setEndAt(end);
        stat.setDurationSeconds(duration);
        stat.setSlotIndex(slotIndex);

        if (slotIndex % 2 == 1) {
            postureStatsRepository.findByProfileIdAndSlotIndex(raw.getProfileId(), slotIndex - 1)
                    .ifPresent(prev -> {
                        double dx = stat.getAvgX() - prev.getAvgX();
                        double dy = stat.getAvgY() - prev.getAvgY();
                        double dz = stat.getAvgZ() - prev.getAvgZ();
                        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                        final double THRESHOLD = 5.0;
                        if (dist <= THRESHOLD) {
                            var sug = new PresetSuggestion();
                            org.springframework.beans.BeanUtils.copyProperties(stat, sug, "id","createdAt");
                            presetSuggestionRepository.save(sug);
                        }
                    });
        }
        return stat;
    }
}
