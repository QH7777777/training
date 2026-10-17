// 修复说明：
// 1. 数字判断：原代码写的是 contains_between(password, 0, 9)，
//    用了整数 0 和 9 而非字符 '0' 和 '9'（ASCII 48~57），导致永远检测不到数字。
// 2. 长度判断：原代码 size() <= 10 方向写反，应为 size() >= 10（至少 10 位）。
// 3. 范围边界：原代码 lower < ch && ch < upper 是开区间，
//    恰好等于边界的字符（如开头的 'A'、结尾的 'Z'）会被漏判，改为 <=。
//
// 自测用例：
//   输入: Ming Li A1b2c3d4e5     -> 预期 VALID   (10位, 含大小写、数字, 不含姓名)
//   输入: Ming Li A1b2c3d4e      -> 预期 INVALID (只有9位, 长度不足)
//   输入: Ming Li a1b2c3d4e5     -> 预期 INVALID (无大写字母)
//   输入: Ming Li A1B2C3D4E5     -> 预期 INVALID (无小写字母)
//   输入: Ming Li Ming1b2c3d4e   -> 预期 INVALID (包含名字 first_name)
//   输入: Ming Li Z9z9z9z9z9     -> 预期 VALID   (边界字符 Z/'9' 也要能匹配)
#include <iostream>
#include <string>

bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        if (lower <= ch && ch <= upper) {
            return true;
        }
    }
    return false;
}

bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    bool length_ok = password.size() >= 10;
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::string first_name, last_name, password;
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
