package siseon.backend.repository.main;

import org.springframework.data.jpa.repository.JpaRepository;
import siseon.backend.domain.main.Profile;
import siseon.backend.domain.main.User;

import java.util.List;

public interface ProfileRepository extends JpaRepository<Profile, Long> {
    List<Profile> findAllByUser(User user);
}