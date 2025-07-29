package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.server.ResponseStatusException;
import org.springframework.http.HttpStatus;
import siseon.backend.domain.User;
import siseon.backend.dto.UserProfile;
import siseon.backend.repository.UserRepository;

@Service
@RequiredArgsConstructor
public class UserService {

    private final UserRepository userRepository;

    @Transactional(readOnly = true)
    public UserProfile getUserProfileByEmail(String email) {
        User user = userRepository.findByEmail(email)
                .orElseThrow(() -> new ResponseStatusException(
                        HttpStatus.NOT_FOUND, "유저를 찾을 수 없습니다."
                ));
        return new UserProfile(
                user.getId(),
                user.getEmail(),
                user.getName(),
                user.getPictureUrl(),
                user.getCreatedAt(),
                user.getUpdatedAt()
        );
    }

    @Transactional
    public void withdrawByEmail(String email) {
        User user = userRepository.findByEmail(email)
                .orElseThrow(() -> new ResponseStatusException(
                        HttpStatus.NOT_FOUND, "유저를 찾을 수 없습니다."
                ));
        userRepository.delete(user);
    }
}
