package siseon.backend.controller;

import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.security.oauth2.jwt.Jwt;
import org.springframework.web.bind.annotation.*;
import siseon.backend.domain.main.User;
import siseon.backend.dto.ProfileResponse;
import siseon.backend.service.ProfileService;
import siseon.backend.service.PushNotificationService;
import siseon.backend.repository.main.UserRepository;

@RestController
@RequestMapping("/api/push")
@RequiredArgsConstructor
public class PushController {

    private final ProfileService profileService;
    private final PushNotificationService pushService;
    private final UserRepository userRepository;

    private User getUserFromJwt(Jwt jwt) {
        return userRepository.findByEmail(jwt.getSubject())
                .orElseThrow(() -> new RuntimeException("사용자를 찾을 수 없습니다."));
    }

    @PostMapping("/register")
    public ResponseEntity<Void> registerToken(
            @AuthenticationPrincipal Jwt jwt,
            @RequestParam Long profileId,
            @RequestParam String fcmToken) {

        User me = getUserFromJwt(jwt);
        profileService.updateFcmToken(profileId, fcmToken, me);
        return ResponseEntity.ok().build();
    }

    @PostMapping("/unregister")
    public ResponseEntity<Void> unregisterToken(
            @AuthenticationPrincipal Jwt jwt,
            @RequestParam Long profileId) {

        User me = getUserFromJwt(jwt);
        profileService.updateFcmToken(profileId, null, me);
        return ResponseEntity.ok().build();
    }

    @PostMapping("/send-test")
    public ResponseEntity<ProfileResponse> sendTest(
            @AuthenticationPrincipal Jwt jwt,
            @RequestParam Long profileId) {

        User me = getUserFromJwt(jwt);
        ProfileResponse dto = profileService.getProfileById(profileId, me);

        if (dto.getFcmToken() != null) {
            pushService.sendPushAsync(
                    dto.getFcmToken(),
                    "테스트 알림",
                    "비동기 FCM 전송이 동작합니다."
            );
        }

        return ResponseEntity.ok(dto);
    }
}
