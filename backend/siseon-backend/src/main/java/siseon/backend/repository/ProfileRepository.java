package siseon.backend.repository;

import org.springframework.data.jpa.repository.JpaRepository;
import siseon.backend.domain.Profile;
import siseon.backend.domain.User;

import java.util.List;

public interface ProfileRepository extends JpaRepository<Profile, Long> {
    List<Profile> findAllByUser(User user);
}
