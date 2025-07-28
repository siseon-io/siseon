package siseon.backend.service;

import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import siseon.backend.domain.Profile;
import siseon.backend.domain.User;
import siseon.backend.dto.ProfileCreateRequest;
import siseon.backend.dto.ProfileResponse;
import siseon.backend.repository.ProfileRepository;

import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Transactional
public class ProfileService {

    private final ProfileRepository profileRepository;

    public ProfileResponse createProfile(ProfileCreateRequest req, User user) {
        Profile profile = Profile.builder()
                .name(req.getName())
                .birthDate(req.getBirthDate())
                .height(req.getHeight())
                .leftVision(req.getLeftVision())
                .rightVision(req.getRightVision())
                .imageUrl(req.getImageUrl())
                .settings(req.getSettings())
                .user(user)
                .build();

        Profile saved = profileRepository.save(profile);
        return toDto(saved);
    }

    public List<ProfileResponse> getProfiles(User user) {
        return profileRepository.findAllByUser(user).stream()
                .map(this::toDto)
                .collect(Collectors.toList());
    }

    public ProfileResponse getProfileById(Long id, User user) {
        Profile profile = getOwnedProfile(id, user);
        return toDto(profile);
    }

    public ProfileResponse updateProfile(Long id, ProfileCreateRequest req, User user) {
        Profile profile = getOwnedProfile(id, user);

        profile.setName(req.getName());
        profile.setBirthDate(req.getBirthDate());
        profile.setHeight(req.getHeight());
        profile.setLeftVision(req.getLeftVision());
        profile.setRightVision(req.getRightVision());
        profile.setImageUrl(req.getImageUrl());
        profile.setSettings(req.getSettings());

        return toDto(profile);
    }

    public void deleteProfile(Long id, User user) {
        Profile profile = getOwnedProfile(id, user);
        profileRepository.delete(profile);
    }

    private Profile getOwnedProfile(Long id, User user) {
        Profile profile = profileRepository.findById(id)
                .orElseThrow(() -> new RuntimeException("해당 프로필을 찾을 수 없습니다."));

        if (!profile.getUser().getId().equals(user.getId())) {
            throw new RuntimeException("본인의 프로필만 접근할 수 있습니다.");
        }

        return profile;
    }

    private ProfileResponse toDto(Profile p) {
        return ProfileResponse.builder()
                .id(p.getId())
                .name(p.getName())
                .birthDate(p.getBirthDate())
                .height(p.getHeight())
                .leftVision(p.getLeftVision())
                .rightVision(p.getRightVision())
                .imageUrl(p.getImageUrl())
                .settings(p.getSettings())
                .build();
    }
}
