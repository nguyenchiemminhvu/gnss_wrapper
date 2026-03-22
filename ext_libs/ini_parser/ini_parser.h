/*
 * MIT License
 *
 * Copyright (c) 2026 nguyenchiemminhvu@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INI_PARSER_H
#define INI_PARSER_H

#include <string>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>

namespace ini_parser
{
    enum class merge_strategy
    {
        OVERWRITE, // new value replaces old
        PRESERVE   // keep old value if present
    };

    class config_error : public std::runtime_error
    {
    public:
        explicit config_error(const std::string& msg) : std::runtime_error(msg) {}
    };

    class io_error : public config_error
    {
    public:
        explicit io_error(const std::string& msg) : config_error(msg) {}
    };

    class parse_error : public config_error
    {
    public:
        explicit parse_error(const std::string& msg) : config_error(msg) {}
    };

    class config_value
    {
    public:
        enum class value_type
        {
            TYPE_NULL,
            TYPE_BOOL,
            TYPE_INT,
            TYPE_DOUBLE,
            TYPE_STRING,
            TYPE_ARRAY,
            TYPE_OBJECT
        };

    private:
        value_type type_;
        
        // Union-like storage for primitive types
        union
        {
            bool bool_value;
            int int_value;
            double double_value;
        };
        
        std::string string_value;
        std::vector<config_value> array_value;
        std::map<std::string, config_value> object_value;

    public:
        // Constructors
        config_value() : type_(value_type::TYPE_NULL), int_value(0) {}
        
        config_value(bool value) : type_(value_type::TYPE_BOOL), bool_value(value) {}
        
        config_value(int value) : type_(value_type::TYPE_INT), int_value(value) {}
        
        config_value(double value) : type_(value_type::TYPE_DOUBLE), double_value(value) {}
        
        config_value(const char* value) : type_(value_type::TYPE_STRING), int_value(0), string_value(value) {}
        
        config_value(const std::string& value) : type_(value_type::TYPE_STRING), int_value(0), string_value(value) {}
        
        config_value(const std::vector<config_value>& value) : type_(value_type::TYPE_ARRAY), int_value(0), array_value(value) {}
        
        config_value(const std::map<std::string, config_value>& value) : type_(value_type::TYPE_OBJECT), int_value(0), object_value(value) {}
        
        // Copy constructor
        config_value(const config_value& other) : type_(other.type_), int_value(0)
        {
            copy_from(other);
        }
        
        // Move constructor
        config_value(config_value&& other) noexcept : type_(other.type_), int_value(0)
        {
            move_from(std::move(other));
        }
        
        // Copy assignment
        config_value& operator=(const config_value& other)
        {
            if (this != &other)
            {
                type_ = other.type_;
                copy_from(other);
            }
            return *this;
        }
        
        // Move assignment
        config_value& operator=(config_value&& other) noexcept
        {
            if (this != &other)
            {
                type_ = other.type_;
                move_from(std::move(other));
            }
            return *this;
        }
        
        ~config_value() = default;
        
        // Type checking
        value_type type() const { return type_; }
        bool is_null() const { return type_ == value_type::TYPE_NULL; }
        bool is_bool() const { return type_ == value_type::TYPE_BOOL; }
        bool is_int() const { return type_ == value_type::TYPE_INT; }
        bool is_double() const { return type_ == value_type::TYPE_DOUBLE; }
        bool is_string() const { return type_ == value_type::TYPE_STRING; }
        bool is_array() const { return type_ == value_type::TYPE_ARRAY; }
        bool is_object() const { return type_ == value_type::TYPE_OBJECT; }
        bool is_number() const { return is_int() || is_double(); }
        
        // Value getters with type checking
        bool as_bool() const
        {
            if (type_ == value_type::TYPE_BOOL)
                return bool_value;
            if (type_ == value_type::TYPE_INT)
                return int_value != 0;
            if (type_ == value_type::TYPE_DOUBLE)
                return double_value != 0.0;
            if (type_ == value_type::TYPE_STRING)
                return !string_value.empty() && string_value != "false" && string_value != "0";
            throw config_error("Cannot convert value to bool");
        }
        
        int as_int() const
        {
            if (type_ == value_type::TYPE_INT)
                return int_value;
            if (type_ == value_type::TYPE_DOUBLE)
                return static_cast<int>(double_value);
            if (type_ == value_type::TYPE_BOOL)
                return bool_value ? 1 : 0;
            if (type_ == value_type::TYPE_STRING)
                return std::stoi(string_value);
            throw config_error("Cannot convert value to int");
        }
        
        double as_double() const
        {
            if (type_ == value_type::TYPE_DOUBLE)
                return double_value;
            if (type_ == value_type::TYPE_INT)
                return static_cast<double>(int_value);
            if (type_ == value_type::TYPE_BOOL)
                return bool_value ? 1.0 : 0.0;
            if (type_ == value_type::TYPE_STRING)
                return std::stod(string_value);
            throw config_error("Cannot convert value to double");
        }
        
        std::string as_string() const
        {
            if (type_ == value_type::TYPE_STRING)
                return string_value;
            if (type_ == value_type::TYPE_INT)
                return std::to_string(int_value);
            if (type_ == value_type::TYPE_DOUBLE)
                return std::to_string(double_value);
            if (type_ == value_type::TYPE_BOOL)
                return bool_value ? "true" : "false";
            if (type_ == value_type::TYPE_NULL)
                return "null";
            throw config_error("Cannot convert complex type to string");
        }
        
        const std::vector<config_value>& as_array() const
        {
            if (type_ != value_type::TYPE_ARRAY)
                throw config_error("Value is not an array");
            return array_value;
        }
        
        std::vector<config_value>& as_array()
        {
            if (type_ != value_type::TYPE_ARRAY)
                throw config_error("Value is not an array");
            return array_value;
        }
        
        const std::map<std::string, config_value>& as_object() const
        {
            if (type_ != value_type::TYPE_OBJECT)
                throw config_error("Value is not an object");
            return object_value;
        }
        
        std::map<std::string, config_value>& as_object()
        {
            if (type_ != value_type::TYPE_OBJECT)
                throw config_error("Value is not an object");
            return object_value;
        }
        
        // Array operations
        size_t array_size() const
        {
            if (type_ != value_type::TYPE_ARRAY)
                throw config_error("Value is not an array");
            return array_value.size();
        }
        
        const config_value& operator[](size_t index) const
        {
            if (type_ != value_type::TYPE_ARRAY)
                throw config_error("Value is not an array");
            if (index >= array_value.size())
                throw config_error("Array index out of bounds");
            return array_value[index];
        }
        
        config_value& operator[](size_t index)
        {
            if (type_ != value_type::TYPE_ARRAY)
                throw config_error("Value is not an array");
            if (index >= array_value.size())
                throw config_error("Array index out of bounds");
            return array_value[index];
        }
        
        void push_back(const config_value& value)
        {
            if (type_ != value_type::TYPE_ARRAY)
                throw config_error("Value is not an array");
            array_value.push_back(value);
        }
        
        // Object operations
        bool has_key(const std::string& key) const
        {
            if (type_ != value_type::TYPE_OBJECT)
                return false;
            return object_value.find(key) != object_value.end();
        }
        
        const config_value& operator[](const std::string& key) const
        {
            if (type_ != value_type::TYPE_OBJECT)
                throw config_error("Value is not an object");
            auto it = object_value.find(key);
            if (it == object_value.end())
                throw config_error("Key not found: " + key);
            return it->second;
        }
        
        config_value& operator[](const std::string& key)
        {
            if (type_ != value_type::TYPE_OBJECT)
                throw config_error("Value is not an object");
            return object_value[key];
        }
        
        // Comparison operators
        bool operator==(const config_value& other) const
        {
            if (type_ != other.type_)
                return false;
                
            switch (type_)
            {
                case value_type::TYPE_NULL:
                    return true;
                case value_type::TYPE_BOOL:
                    return bool_value == other.bool_value;
                case value_type::TYPE_INT:
                    return int_value == other.int_value;
                case value_type::TYPE_DOUBLE:
                    return double_value == other.double_value;
                case value_type::TYPE_STRING:
                    return string_value == other.string_value;
                case value_type::TYPE_ARRAY:
                    return array_value == other.array_value;
                case value_type::TYPE_OBJECT:
                    return object_value == other.object_value;
            }
            return false;
        }
        
        bool operator!=(const config_value& other) const
        {
            return !(*this == other);
        }
        
    private:
        void copy_from(const config_value& other)
        {
            switch (other.type_)
            {
                case value_type::TYPE_BOOL:
                    bool_value = other.bool_value;
                    break;
                case value_type::TYPE_INT:
                    int_value = other.int_value;
                    break;
                case value_type::TYPE_DOUBLE:
                    double_value = other.double_value;
                    break;
                case value_type::TYPE_STRING:
                    string_value = other.string_value;
                    break;
                case value_type::TYPE_ARRAY:
                    array_value = other.array_value;
                    break;
                case value_type::TYPE_OBJECT:
                    object_value = other.object_value;
                    break;
                default:
                    break;
            }
        }
        
        void move_from(config_value&& other) noexcept
        {
            switch (other.type_)
            {
                case value_type::TYPE_BOOL:
                    bool_value = other.bool_value;
                    break;
                case value_type::TYPE_INT:
                    int_value = other.int_value;
                    break;
                case value_type::TYPE_DOUBLE:
                    double_value = other.double_value;
                    break;
                case value_type::TYPE_STRING:
                    string_value = std::move(other.string_value);
                    break;
                case value_type::TYPE_ARRAY:
                    array_value = std::move(other.array_value);
                    break;
                case value_type::TYPE_OBJECT:
                    object_value = std::move(other.object_value);
                    break;
                default:
                    break;
            }
        }
    };

    class base_ini_parser
    {
    protected:
        std::map<std::string, config_value> data_;
        
    public:
        base_ini_parser() = default;
        virtual ~base_ini_parser() = default;
        virtual bool load(const std::string &source, merge_strategy strategy = merge_strategy::OVERWRITE) = 0;
        virtual bool save(const std::string &destination) const = 0;

        virtual void merge(const base_ini_parser &other, merge_strategy strategy = merge_strategy::OVERWRITE)
        {
            for (const auto& pair : other.data_)
            {
                if (strategy == merge_strategy::OVERWRITE || !has(pair.first))
                {
                    data_[pair.first] = pair.second;
                }
            }
        }

        bool has(const std::string& key) const
        {
            return data_.find(key) != data_.end();
        }
        
        config_value get(const std::string& key) const
        {
            auto it = data_.find(key);
            if (it == data_.end())
                throw config_error("Key not found: " + key);
            return it->second;
        }
        
        config_value get(const std::string& key, const config_value& default_value) const
        {
            auto it = data_.find(key);
            return (it != data_.end()) ? it->second : default_value;
        }
        
        void set(const std::string& key, const config_value& value)
        {
            data_[key] = value;
        }
        
        template<typename T>
        void set(const std::string& key, const T& value)
        {
            data_[key] = config_value(value);
        }
        
        bool remove(const std::string& key)
        {
            return data_.erase(key) > 0;
        }
        
        void clear()
        {
            data_.clear();
        }
        
        // Get all keys
        std::vector<std::string> get_keys() const
        {
            std::vector<std::string> keys;
            keys.reserve(data_.size());
            for (const auto& pair : data_)
            {
                keys.push_back(pair.first);
            }
            return keys;
        }
        
        size_t size() const
        {
            return data_.size();
        }
        
        bool empty() const
        {
            return data_.empty();
        }
        
        // Section support (using dot notation: "section.key")
        bool has_section(const std::string& section) const
        {
            std::string prefix = section + ".";
            for (const auto& pair : data_)
            {
                if (pair.first.compare(0, prefix.length(), prefix) == 0)
                    return true;
            }
            return false;
        }
        
        std::vector<std::string> get_section_keys(const std::string& section) const
        {
            std::vector<std::string> keys;
            std::string prefix = section + ".";
            for (const auto& pair : data_)
            {
                if (pair.first.compare(0, prefix.length(), prefix) == 0)
                {
                    keys.push_back(pair.first.substr(prefix.length()));
                }
            }
            return keys;
        }
        
        std::map<std::string, config_value> get_section(const std::string& section) const
        {
            std::map<std::string, config_value> section_data;
            std::string prefix = section + ".";
            for (const auto& pair : data_)
            {
                if (pair.first.compare(0, prefix.length(), prefix) == 0)
                {
                    section_data[pair.first.substr(prefix.length())] = pair.second;
                }
            }
            return section_data;
        }
        
        // Iterator access
        typedef std::map<std::string, config_value>::const_iterator const_iterator;
        typedef std::map<std::string, config_value>::iterator iterator;
        
        const_iterator begin() const { return data_.begin(); }
        const_iterator end() const { return data_.end(); }
        iterator begin() { return data_.begin(); }
        iterator end() { return data_.end(); }
    };

    // INI file parser (supports sections, key=value pairs, comments with # or ;)
    class ini_parser : public base_ini_parser
    {
    public:
        bool load(const std::string &source, merge_strategy strategy = merge_strategy::OVERWRITE) override
        {
            std::ifstream file(source);
            if (!file.is_open())
            {
                throw io_error("Failed to open INI file: " + source);
            }
            
            // If PRESERVE strategy, store existing data
            std::map<std::string, config_value> backup_data;
            if (strategy == merge_strategy::PRESERVE)
            {
                backup_data = data_;
            }
            else
            {
                data_.clear();
            }
            
            std::string line;
            std::string current_section;
            size_t line_number = 0;
            
            try
            {
                while (std::getline(file, line))
                {
                    ++line_number;
                    line = trim(line);
                    
                    // Skip empty lines and comments
                    if (line.empty() || is_comment(line))
                        continue;
                    
                    // Check for section header
                    std::string section;
                    if (is_section_header(line, section))
                    {
                        current_section = section;
                        continue;
                    }
                    
                    // Parse key-value pair
                    std::string key, value;
                    if (parse_key_value(line, key, value))
                    {
                        std::string full_key = current_section.empty() ? key : current_section + "." + key;
                        
                        // Apply merge strategy
                        if (strategy == merge_strategy::PRESERVE && backup_data.find(full_key) != backup_data.end())
                        {
                            data_[full_key] = backup_data[full_key];
                        }
                        else
                        {
                            data_[full_key] = parse_value(value);
                        }
                    }
                    else
                    {
                        throw parse_error("Invalid INI syntax at line " + std::to_string(line_number) + ": " + line);
                    }
                }
                
                file.close();
                return true;
            }
            catch (const std::exception& e)
            {
                file.close();
                throw parse_error("Error parsing INI file at line " + std::to_string(line_number) + ": " + e.what());
            }
        }

        bool save(const std::string &destination) const override
        {
            std::ofstream file(destination);
            if (!file.is_open())
            {
                throw io_error("Failed to open INI file for writing: " + destination);
            }
            
            try
            {
                // Group keys by section
                std::map<std::string, std::vector<std::pair<std::string, config_value>>> sections;
                
                for (const auto& pair : data_)
                {
                    size_t dot_pos = pair.first.find('.');
                    if (dot_pos != std::string::npos)
                    {
                        std::string section = pair.first.substr(0, dot_pos);
                        std::string key = pair.first.substr(dot_pos + 1);
                        sections[section].emplace_back(key, pair.second);
                    }
                    else
                    {
                        sections[""].emplace_back(pair.first, pair.second);
                    }
                }
                
                // Write global keys first (keys without section)
                if (sections.find("") != sections.end() && !sections[""].empty())
                {
                    for (const auto& kv : sections[""])
                    {
                        write_key_value(file, kv.first, kv.second);
                    }
                    file << "\n";
                }
                
                // Write sections
                for (const auto& section_pair : sections)
                {
                    if (section_pair.first.empty())
                        continue;
                    
                    file << "[" << section_pair.first << "]\n";
                    for (const auto& kv : section_pair.second)
                    {
                        write_key_value(file, kv.first, kv.second);
                    }
                    file << "\n";
                }
                
                file.close();
                return true;
            }
            catch (const std::exception& e)
            {
                file.close();
                throw io_error("Error writing INI file: " + std::string(e.what()));
            }
        }
        
    private:
        // Trim whitespace from both ends of string
        std::string trim(const std::string& str) const
        {
            size_t start = 0;
            size_t end = str.length();
            
            while (start < end && std::isspace(static_cast<unsigned char>(str[start])))
                ++start;
            
            while (end > start && std::isspace(static_cast<unsigned char>(str[end - 1])))
                --end;
            
            return str.substr(start, end - start);
        }
        
        // Check if line is a comment
        bool is_comment(const std::string& line) const
        {
            return !line.empty() && (line[0] == '#' || line[0] == ';');
        }
        
        // Check if line is a section header and extract section name
        bool is_section_header(const std::string& line, std::string& section) const
        {
            if (line.length() < 3 || line[0] != '[')
                return false;
            
            size_t close_bracket = line.find(']');
            if (close_bracket == std::string::npos)
                return false;
            
            section = trim(line.substr(1, close_bracket - 1));
            
            // Validate section name (no dots allowed as they're used as separators)
            if (section.empty() || section.find('.') != std::string::npos)
                return false;
            
            return true;
        }
        
        // Parse key=value or key:value pair
        bool parse_key_value(const std::string& line, std::string& key, std::string& value) const
        {
            size_t separator_pos = line.find('=');
            if (separator_pos == std::string::npos)
            {
                separator_pos = line.find(':');
            }
            
            if (separator_pos == std::string::npos || separator_pos == 0)
                return false;
            
            key = trim(line.substr(0, separator_pos));
            value = trim(line.substr(separator_pos + 1));
            
            // Remove quotes from value if present
            if (value.length() >= 2 && 
                ((value.front() == '"' && value.back() == '"') ||
                 (value.front() == '\'' && value.back() == '\'')))
            {
                value = value.substr(1, value.length() - 2);
            }
            
            return !key.empty();
        }
        
        // Parse value string and infer type
        config_value parse_value(const std::string& value_str) const
        {
            if (value_str.empty())
                return config_value("");
            
            // Check for boolean values
            std::string lower_value = to_lower(value_str);
            if (lower_value == "true" || lower_value == "yes" || lower_value == "on")
                return config_value(true);
            if (lower_value == "false" || lower_value == "no" || lower_value == "off")
                return config_value(false);
            
            // Check for null
            if (lower_value == "null" || lower_value == "nil" || lower_value == "none")
                return config_value();
            
            // Try to parse as number
            try
            {
                // Check if it contains a decimal point
                if (value_str.find('.') != std::string::npos)
                {
                    size_t idx;
                    double d = std::stod(value_str, &idx);
                    if (idx == value_str.length())
                        return config_value(d);
                }
                else
                {
                    size_t idx;
                    int i = std::stoi(value_str, &idx);
                    if (idx == value_str.length())
                        return config_value(i);
                }
            }
            catch (...)
            {
                // Not a number, treat as string
            }
            
            // Default to string
            return config_value(value_str);
        }
        
        // Convert string to lowercase
        std::string to_lower(const std::string& str) const
        {
            std::string result = str;
            for (char& c : result)
            {
                c = std::tolower(static_cast<unsigned char>(c));
            }
            return result;
        }
        
        // Write key-value pair to file
        void write_key_value(std::ofstream& file, const std::string& key, const config_value& value) const
        {
            file << key << " = ";
            
            if (value.is_string())
            {
                // Quote strings if they contain special characters or spaces
                const std::string& str_val = value.as_string();
                if (needs_quoting(str_val))
                    file << "\"" << escape_string(str_val) << "\"";
                else
                    file << str_val;
            }
            else if (value.is_int())
            {
                file << value.as_int();
            }
            else if (value.is_double())
            {
                file << value.as_double();
            }
            else if (value.is_bool())
            {
                file << (value.as_bool() ? "true" : "false");
            }
            else if (value.is_null())
            {
                file << "null";
            }
            else
            {
                // For complex types (arrays, objects), convert to string representation
                file << "\"[complex value]\"";
            }
            
            file << "\n";
        }
        
        // Check if string needs quoting
        bool needs_quoting(const std::string& str) const
        {
            if (str.empty())
                return true;
            
            // Check for whitespace, special characters, or comment markers
            for (char c : str)
            {
                if (std::isspace(static_cast<unsigned char>(c)) || 
                    c == '#' || c == ';' || c == '=' || c == ':' || c == '[' || c == ']')
                {
                    return true;
                }
            }
            
            return false;
        }
        
        // Escape special characters in strings
        std::string escape_string(const std::string& str) const
        {
            std::string result;
            result.reserve(str.length());
            
            for (char c : str)
            {
                switch (c)
                {
                    case '"':  result += "\\\""; break;
                    case '\\': result += "\\\\"; break;
                    case '\n': result += "\\n"; break;
                    case '\r': result += "\\r"; break;
                    case '\t': result += "\\t"; break;
                    default:   result += c; break;
                }
            }
            
            return result;
        }
    };
}

#endif // INI_PARSER_H
