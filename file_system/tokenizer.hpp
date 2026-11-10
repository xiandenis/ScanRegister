

#ifndef UTIL_TOKENIZE_HEADER
#define UTIL_TOKENIZE_HEADER

#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include "strings.hpp"

namespace util {


	class Tokenizer : public std::vector<std::string>
	{
	public:

		void split(std::string const& str, char delim = ' ',
			bool keep_empty = false);

		void split(std::string const& str, char delim1, char delim2,
			bool keep_empty);

		void split(std::string const& str, char delim1, char delim2, char delim3,
			bool keep_empty);

		void split(std::string const& str, char delim1, char delim2, char delim3, char delim4,
			bool keep_empty);

		void parse_cmd(std::string const& str);

		std::string concat(std::size_t pos, std::size_t num = 0) const;

		template <typename T>
		T get_as(std::size_t pos) const;
	};



	inline void
		Tokenizer::split(std::string const& str, char delim, bool keep_empty)
	{
		this->clear();
		std::size_t new_tok = 0;
		std::size_t cur_pos = 0;
		for (; cur_pos < str.size(); ++cur_pos)
			if (str[cur_pos] == delim)
			{
				std::string token = str.substr(new_tok, cur_pos - new_tok);
				if (keep_empty || !token.empty())
					this->push_back(token);
				new_tok = cur_pos + 1;
			}

		if (keep_empty || new_tok < str.size())
			this->push_back(str.substr(new_tok));
	}

	inline void
		Tokenizer::split(std::string const& str, char delim1, char delim2, bool keep_empty)
	{
		this->clear();
		std::size_t new_tok = 0;
		std::size_t cur_pos = 0;
		for (; cur_pos < str.size(); ++cur_pos)
			if (str[cur_pos] == delim1 || str[cur_pos] == delim2)
			{
				std::string token = str.substr(new_tok, cur_pos - new_tok);
				if (keep_empty || !token.empty())
					this->push_back(token);
				new_tok = cur_pos + 1;
			}

		if (keep_empty || new_tok < str.size())
			this->push_back(str.substr(new_tok));
	}

	inline void
		Tokenizer::split(std::string const& str, char delim1, char delim2, char delim3, bool keep_empty)
	{
		this->clear();
		std::size_t new_tok = 0;
		std::size_t cur_pos = 0;
		for (; cur_pos < str.size(); ++cur_pos)
			if (str[cur_pos] == delim1 || str[cur_pos] == delim2 || str[cur_pos] == delim3)
			{
				std::string token = str.substr(new_tok, cur_pos - new_tok);
				if (keep_empty || !token.empty())
					this->push_back(token);
				new_tok = cur_pos + 1;
			}

		if (keep_empty || new_tok < str.size())
			this->push_back(str.substr(new_tok));
	}

	inline void
		Tokenizer::split(std::string const& str, char delim1, char delim2, char delim3, char delim4, bool keep_empty)
	{
		this->clear();
		std::size_t new_tok = 0;
		std::size_t cur_pos = 0;
		for (; cur_pos < str.size(); ++cur_pos)
			if (str[cur_pos] == delim1 || str[cur_pos] == delim2 || str[cur_pos] == delim3 || str[cur_pos] == delim4)
			{
				std::string token = str.substr(new_tok, cur_pos - new_tok);
				if (keep_empty || !token.empty())
					this->push_back(token);
				new_tok = cur_pos + 1;
			}

		if (keep_empty || new_tok < str.size())
			this->push_back(str.substr(new_tok));
	}


	inline void
		Tokenizer::parse_cmd(std::string const& str)
	{
		this->clear();
		bool in_quote = false;
		std::string token;
		for (std::size_t i = 0; i < str.size(); ++i)
		{
			char chr = str[i];

			if (chr == ' ' && !in_quote)
			{
				this->push_back(token);
				token.clear();
			}
			else if (chr == '"')
				in_quote = !in_quote;
			else
				token.push_back(chr);
		}
		this->push_back(token);
	}


	inline std::string
		Tokenizer::concat(std::size_t pos, std::size_t num) const
	{
		std::stringstream ss;
		std::size_t max = (num == 0
			? this->size()
			: std::min(pos + num, this->size()));

		for (std::size_t i = pos; i < max; ++i)
		{
			ss << (i == pos ? "" : " ");
			ss << this->at(i);
		}

		return ss.str();
	}


	template <typename T>
	inline T
		Tokenizer::get_as(std::size_t pos) const
	{
		return strings::convert<T>(this->at(pos));
	}

};

#endif /* UTIL_TOKENIZE_HEADER */
