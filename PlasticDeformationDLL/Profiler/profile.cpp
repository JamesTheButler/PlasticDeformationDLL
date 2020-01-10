#include "profile.h"

namespace realm
{

Profile* profile = nullptr;
Profile* GetProfiler()
{
	if (profile == nullptr)
		profile = new Profile;
	return profile;
}

void DeallocateProfiler()
{
	if (profile)
		delete profile;
	profile = nullptr;
}


// Total time reference
double ProfileNode::mGlobalTime = 0;

/// Reference frame counter
unsigned int ProfileNode::mReferenceCount = 0;

/// depth of profile
int ProfileNode::mFilter = 0;
} // namespace realm
