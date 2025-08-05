package siseon.backend.listener;

import lombok.RequiredArgsConstructor;
import org.springframework.batch.core.ExitStatus;
import org.springframework.batch.core.StepExecution;
import org.springframework.batch.core.StepExecutionListener;
import org.springframework.stereotype.Component;
import siseon.backend.repository.main.RawPostureRepository;

import java.time.LocalDateTime;

@Component
@RequiredArgsConstructor
public class RawPostureStepListener implements StepExecutionListener {

    private final RawPostureRepository rawPostureRepository;

    @Override
    public void beforeStep(StepExecution stepExecution) {
        try {
            var params = stepExecution.getJobParameters();
            String startStr = params.getString("startTime");
            String endStr   = params.getString("endTime");

            if (startStr == null || endStr == null) {
                System.err.println("[ERROR] startTime 또는 endTime 파라미터 누락 → Step 종료");
                stepExecution.setTerminateOnly();
                return;
            }

            LocalDateTime start = LocalDateTime.parse(startStr);
            LocalDateTime end   = LocalDateTime.parse(endStr);

            long count = rawPostureRepository.countByCollectedAtBetween(start, end);
            if (count == 0) {
                System.out.printf("[SKIP] [%s ~ %s] 구간에 raw_postures 없음 → Step 종료%n", start, end);
                stepExecution.setTerminateOnly();
            } else {
                System.out.printf("[INFO] [%s ~ %s] 구간에 raw_postures %d건 존재 → Step 진행%n", start, end, count);
            }

        } catch (Exception e) {
            System.err.println("[EXCEPTION] RawPostureStepListener.beforeStep 실패: " + e.getMessage());
            stepExecution.setTerminateOnly();
        }
    }

    @Override
    public ExitStatus afterStep(StepExecution stepExecution) {
        return stepExecution.getExitStatus();
    }
}