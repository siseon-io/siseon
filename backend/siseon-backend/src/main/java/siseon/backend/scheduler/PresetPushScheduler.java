package siseon.backend.scheduler;

import lombok.RequiredArgsConstructor;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;
import siseon.backend.repository.batch.PresetSuggestionRepository;
import siseon.backend.repository.main.ProfileRepository;
import siseon.backend.service.PushNotificationService;
import siseon.backend.domain.main.Profile;

@Component
@RequiredArgsConstructor
public class PresetPushScheduler {

    private final PresetSuggestionRepository sugRepo;
    private final ProfileRepository profileRepo;
    private final PushNotificationService pushService;

    @Scheduled(fixedDelay = 300_000)
    public void pushPresetSuggestions() {
        sugRepo.findBySatisfiedFalse().forEach(sug -> {
            Profile profile = profileRepo.findById(sug.getProfileId())
                    .orElseThrow(() -> new IllegalStateException(
                            "프로필을 찾을 수 없습니다: " + sug.getProfileId()));
            String token = profile.getFcmToken();

            String title = "프리셋 저장 제안";
            String body  = "이 자세로 프리셋을 저장하시겠어요?";

            pushService.sendPushAsync(token, title, body);

            sug.setSatisfied(true);
            sugRepo.save(sug);
        });
    }
}
