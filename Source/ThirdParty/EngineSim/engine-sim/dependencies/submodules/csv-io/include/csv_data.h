#ifndef ATG_CSV_IO_CSV_DATA_H
#define ATG_CSV_IO_CSV_DATA_H

#include <istream>

namespace atg_csv {
    class CsvData {
        public:
            enum class ErrorCode {
                Success,
                CouldNotOpenFile,
                InconsistentColumnCount,
                UnexpectedCharacter,
                UnexpectedEndOfFile,
                Unknown
            };

            struct ErrorInfo {
                const char *msg = "";
                int line = -1;
                int column = -1;
            };

            struct CharBuffer {
                char *buffer = nullptr;
                int writeIndex = 0;
                int bufferSize = 0;

                void initialize(int bufferSize);
                void write(char c);
                void reset();
                void destroy();
            };

        public:
            CsvData();
            ~CsvData();

            void initialize(int initElements = 4, int initCapacity = 1024);
            void write(const char *entry);
            void destroy();

            ErrorCode loadCsv(const char *fname, ErrorInfo *err = nullptr, char del = ',');
            ErrorCode writeCsv(const char *fname, ErrorInfo *err = nullptr, char del = ',');

            inline const char *readEntry(int row, int col) const {
                return m_data[row * m_columns + col] + m_buffer;
            }

            inline void setWriteEntry(int row, int col) {
                m_writeEntry = row * m_columns + col;
            }

            inline void setWritePosition(int row, int col) {
                m_writePosition = m_data[row * m_columns + col];
            }

        public:
            int m_rows = 0;
            int m_columns = 0;

        protected:
            void resize(size_t newCapacity);
            void resizeElements(size_t elementCapacity);

            ErrorCode loadCsv(std::istream &is, ErrorInfo *err, char del);

        protected:
            size_t *m_data = nullptr;
            size_t m_entryCapacity = 0;

            char *m_buffer = nullptr;
            size_t m_bufferCapacity = 0;

            size_t m_writePosition = 0;
            int m_writeEntry = 0;
    };
} /* namespace atg_csv */

#endif /* ATG_CSV_IO_CSV_DATA_H */
