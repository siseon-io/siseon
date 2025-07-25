package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.security.core.annotation.AuthenticationPrincipal;
import org.springframework.stereotype.Service;
import siseon.backend.domain.User;
import siseon.backend.dto.UserProfile;
import siseon.backend.repository.UserRepository;

@Service
@RequiredArgsConstructor
public class UserService {

    private final UserRepository userRepository;

    public UserProfile getUserProfileByEmail(String email) {
        User u = userRepository.findByEmail(email)
                .orElseThrow(() -> new RuntimeException("유저를 찾을 수 없습니다."));
        return new UserProfile(
                u.getId(), u.getEmail(), u.getName(),
                u.getPictureUrl(), u.getCreatedAt(), u.getUpdatedAt()
        );
    }
}
