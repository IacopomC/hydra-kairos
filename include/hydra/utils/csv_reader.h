#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * @file csv_reader.h
 * @brief Reader that loads a delimited text file fully into memory and exposes its rows
 *        by index and its columns by header name. Lookups by header are backed by a
 *        shared name-to-column-index map so individual rows can resolve entries on their
 *        own.
 */

namespace hydra {

/// @brief Loads a CSV file and provides indexed and header-keyed access to its contents.
class CsvReader {
 public:
  /// @brief Construct an unloaded reader; call setup() before querying.
  CsvReader() = default;
  /// @brief Construct and immediately load a file.
  /// @param file_name Path to the file to read.
  /// @param separator Field delimiter character.
  /// @param skip_first_line If true, treat the first line as a header row.
  explicit CsvReader(const std::string& file_name,
                     char separator = ',',
                     bool skip_first_line = true);
  virtual ~CsvReader() = default;

  /// @brief Load a CSV file, replacing any previously loaded data.
  /// @param file_name The name of the csv file to read.
  /// @param separator The separator used in the csv file.
  /// @param skip_first_line If true, the first line is parsed as column headers.
  /// @return True if the file was read successfully, false otherwise.
  bool setup(const std::string& file_name,
             char separator = ',',
             bool skip_first_line = true);
  /// @brief Report whether a file has been successfully loaded.
  bool isSetup() const { return is_setup_; }
  /// @brief Implicit bool conversion mirroring isSetup().
  operator bool() const { return isSetup(); }

  /// @brief One parsed CSV row, with the column entries plus a shared header index for
  /// resolving entries by column name.
  struct Row : std::vector<std::string> {
   public:
    /// @brief Construct an empty row with no header mapping.
    Row() = default;
    /// @brief Construct a row from parsed fields and a shared header index.
    /// @param row The field values for this row.
    /// @param _header_to_index Shared map from header name to column index.
    Row(const std::vector<std::string>& row,
        std::shared_ptr<std::unordered_map<std::string, size_t>> _header_to_index)
        : std::vector<std::string>(row), header_to_index(std::move(_header_to_index)) {}
    /// @brief Look up this row's entry for a column by header name.
    /// @param header Column header to resolve.
    /// @return The entry, or the first field if the header is unknown.
    const std::string& getEntry(const std::string& header) const;

   private:
    std::shared_ptr<std::unordered_map<std::string, size_t>> header_to_index;  ///< Shared header-to-column map.
  };

  /// @brief Number of data rows, excluding any header line.
  size_t numRows() const;
  /// @brief Report whether the loaded file contains the given header.
  bool hasHeader(const std::string& header) const;
  /// @brief Report whether the loaded file contains every header in the list.
  bool hasHeaders(const std::vector<std::string>& headers) const;
  /// @brief Access the list of column headers.
  const std::vector<std::string>& getHeaders() const;
  /// @brief Access all data rows.
  const std::vector<Row>& getRows() const;
  /// @brief Access a single row by index; out-of-range indices log a warning.
  const Row& getRow(size_t row) const;
  /// @brief Look up an entry by header name and row index.
  /// @param header Column header to resolve.
  /// @param row Row index.
  /// @return The entry, or the first field if the header or row is invalid.
  const std::string& getEntry(const std::string& header, size_t row) const;

  /// @brief Require the loaded file to contain the given headers, logging an error for
  /// any that are missing.
  /// @param headers List of headers that are required.
  /// @return True if all required headers are present, false otherwise.
  bool checkRequiredHeaders(const std::vector<std::string>& headers) const;

  /// @brief Check for the given headers, logging a warning for any that are missing.
  /// @param headers List of headers that are optional.
  void checkOptionalHeaders(const std::vector<std::string>& headers) const;

 private:
  bool is_setup_ = false;            ///< True once a file has been loaded.
  std::string loaded_file_name_;     ///< Path of the most recently loaded file.

  std::vector<std::string> headers_;  ///< Column header names.
  std::vector<Row> rows_;             ///< Parsed data rows.
  std::shared_ptr<std::unordered_map<std::string, size_t>> header_to_index_;  ///< Header-to-column index map shared with rows.

  /// @brief Return the subset of the given headers absent from the loaded file.
  std::vector<std::string> missingHeaders(const std::vector<std::string>& headers) const;
};

}  // namespace hydra
