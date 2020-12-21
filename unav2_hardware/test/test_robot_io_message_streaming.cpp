#include <gtest/gtest.h>
#include <robot_io/message_streaming.h>
#include <robot_io/interface.h>
#include <vector>

void print_container(const std::vector<uint8_t> &c) {
  for (auto &i : c) {
    std::cout << i << " ";
  }
  std::cout << '\n';
}

void check_decoded_data(const std::vector<uint8_t> &decoded, const std::vector<uint8_t> test_data, size_t size) {
  ASSERT_EQ(decoded.size(), size);
  for (size_t i = 0; i < size; i++) {
    ASSERT_EQ(decoded.at(i), test_data.at(i));
  }
}

TEST(Robot_IO, streaming_encode_decode) {

  const std::vector<uint8_t> test_data{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  auto encoded = robot_io::MessageStreaming::encode(test_data);
  ASSERT_EQ(encoded.size(), test_data.size() + robot_io::MessageStreaming::SIZE_FIELD_LENGHT);

  auto size = encoded.front();
  ASSERT_EQ(size, (uint8_t)test_data.size());

  std::vector<uint8_t> decoded;
  ASSERT_TRUE(robot_io::MessageStreaming::decode(encoded, decoded));

  check_decoded_data(decoded, test_data, test_data.size());

  ASSERT_TRUE(encoded.size() == 0);
}

TEST(Robot_IO, streaming_decoded_payload_removal) {
  const std::vector<uint8_t> data1{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  const std::vector<uint8_t> data2{9, 8, 7, 6};
  const size_t size1 = data1.size();
  const size_t size2 = data2.size();

  // construct the test encoded packet
  // size1 | data1 | size2 | data2
  std::vector<uint8_t> encoded_test_data;

  encoded_test_data.push_back(static_cast<uint8_t>(size1));
  encoded_test_data.insert(encoded_test_data.end(), data1.cbegin(), data1.cend());
  encoded_test_data.push_back(static_cast<uint8_t>(size2));
  encoded_test_data.insert(encoded_test_data.end(), data2.cbegin(), data2.cend());

  std::vector<uint8_t> decoded;
  ASSERT_TRUE(robot_io::MessageStreaming::decode(encoded_test_data, decoded));

  check_decoded_data(decoded, data1, size1);

  // check that the encoded data vector only contains the second packet
  ASSERT_TRUE(encoded_test_data.size() == size2 + robot_io::MessageStreaming::SIZE_FIELD_LENGHT);
  std::vector<uint8_t> decoded2;
  // print_container(encoded_test_data);
  ASSERT_TRUE(robot_io::MessageStreaming::decode(encoded_test_data, decoded2));
  check_decoded_data(decoded2, data2, size2);
}

TEST(Robot_IO, streaming_decode_incomplete_packet_fail) {
  const std::vector<uint8_t> data{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  // force an invalid size
  const size_t wrong_size = data.size() + 2;

  // construct the test incomplete encoded packet
  // size1 | data1
  std::vector<uint8_t> encoded_test_data;

  encoded_test_data.push_back(static_cast<uint8_t>(wrong_size));
  encoded_test_data.insert(encoded_test_data.end(), data.cbegin(), data.cend());

  std::vector<uint8_t> decoded;
  ASSERT_FALSE(robot_io::MessageStreaming::decode(encoded_test_data, decoded));

  // check that no data have been removed from the encoded vector
  ASSERT_TRUE(encoded_test_data.size() == data.size() + robot_io::MessageStreaming::SIZE_FIELD_LENGHT);
}

