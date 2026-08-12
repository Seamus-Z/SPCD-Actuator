#include "middleware/communication/binary_link_policy.h"

#include "gtest/gtest.h"

namespace middleware::communication
{
namespace
{

TEST(BinaryLinkPolicyTest, AcceptsOnlyAddressedDataEnvelope)
{
  constexpr uint16_t kCommandId = 0x101;
  constexpr size_t kMinimumLength = sizeof(protocol::xt_can::Header) + 1;

  EXPECT_TRUE(IsCommandFrameEnvelope(true, kCommandId, true, kMinimumLength,
                                     kCommandId));
  EXPECT_FALSE(IsCommandFrameEnvelope(false, kCommandId, true, kMinimumLength,
                                      kCommandId));
  EXPECT_FALSE(IsCommandFrameEnvelope(true, kCommandId + 1, true,
                                      kMinimumLength, kCommandId));
  EXPECT_FALSE(IsCommandFrameEnvelope(true, kCommandId, false,
                                      kMinimumLength, kCommandId));
  EXPECT_FALSE(IsCommandFrameEnvelope(true, kCommandId, true,
                                      kMinimumLength - 1, kCommandId));
}

TEST(BinaryLinkPolicyTest, RejectsEveryInvalidProtocolHeaderField)
{
  const protocol::xt_can::Header valid{
      protocol::xt_can::kMagic, protocol::xt_can::kVersion, protocol::xt_can::kTypeCmd, 7};
  EXPECT_TRUE(IsCommandProtocolHeader(valid));

  auto invalid = valid;
  invalid.magic ^= 1;
  EXPECT_FALSE(IsCommandProtocolHeader(invalid));

  invalid = valid;
  invalid.ver += 1;
  EXPECT_FALSE(IsCommandProtocolHeader(invalid));

  invalid = valid;
  invalid.type = protocol::xt_can::kTypeTel;
  EXPECT_FALSE(IsCommandProtocolHeader(invalid));
}

}  // namespace
}  // namespace middleware::communication
