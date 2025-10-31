#include <unity.h>

#include "input_match.hpp"

static void test_literal_match() {
    InputMatch matcher;
    bool called = false;
    matcher.match("12#", [&](const InputMatch::Result&) {
        called = true;
    });
    TEST_ASSERT_FALSE(matcher.consume('1'));
    TEST_ASSERT_FALSE(called);
    TEST_ASSERT_FALSE(matcher.consume('2'));
    TEST_ASSERT_FALSE(called);
    TEST_ASSERT_TRUE(matcher.consume('#'));
    TEST_ASSERT_TRUE(called);
    TEST_ASSERT_TRUE(matcher.buffer().empty());
}

static void test_numeric_capture_and_leading_zero() {
    InputMatch matcher;
    InputMatch::Result captured{};
    matcher.match("Bnnn#", [&](const InputMatch::Result& res) {
        captured = res;
    });
    matcher.consume('B');
    matcher.consume('0');
    matcher.consume('1');
    matcher.consume('2');
    matcher.consume('#');
    TEST_ASSERT_TRUE(captured.has_number);
    TEST_ASSERT_EQUAL(12, captured.number);
    TEST_ASSERT_TRUE(captured.leading_zero);
}

static void test_buffer_reset_with_star() {
    InputMatch matcher;
    int hits = 0;
    matcher.match("A1", [&](const InputMatch::Result&) {
        hits++;
    });
    matcher.consume('A');
    matcher.consume('*'); // should reset buffer
    TEST_ASSERT_EQUAL(0, hits);
    TEST_ASSERT_TRUE(matcher.buffer().empty());
    matcher.consume('A');
    matcher.consume('1');
    TEST_ASSERT_EQUAL(1, hits);
    TEST_ASSERT_TRUE(matcher.buffer().empty());
}

static void test_star_literal_pattern() {
    InputMatch matcher;
    bool called = false;
    matcher.match("*12", [&](const InputMatch::Result&) {
        called = true;
    });
    matcher.consume('*');
    matcher.consume('1');
    matcher.consume('2');
    TEST_ASSERT_TRUE(called);
    TEST_ASSERT_TRUE(matcher.buffer().empty());
}

void setUp() {}

void tearDown() {}

#ifdef ARDUINO
void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_literal_match);
    RUN_TEST(test_numeric_capture_and_leading_zero);
    RUN_TEST(test_buffer_reset_with_star);
    RUN_TEST(test_star_literal_pattern);
    UNITY_END();
}

void loop() {}
#else
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_literal_match);
    RUN_TEST(test_numeric_capture_and_leading_zero);
    RUN_TEST(test_buffer_reset_with_star);
    RUN_TEST(test_star_literal_pattern);
    return UNITY_END();
}
#endif
