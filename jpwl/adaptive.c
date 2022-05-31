#include "adaptive.h"
#include <stdio.h>
#include <memory.h>
#include <Windows.h>
#include <WinBase.h>

__declspec(dllexport)
void select_params_adaptive(float buffer_errors, float recovered_tiles, float min_tiles_percent, jpwl_enc_params* params) {
	static float prev_rec_errors[WINDOW_SIZE] = { 0, 0, 0, 0 };
	static float prev_rec_tiles[WINDOW_SIZE] = { 0, 0, 0, 0 };
	static int prev_idx = 0, window_cnt = 0;

	int jpwl_idx = 0;
	for (int i = 0; i < JPWL_CODES; i++) {
		if (jpwl_codes[i] == params->wcoder_data) {
			jpwl_idx = i;
			break;
		}
	}
	prev_rec_errors[prev_idx] = buffer_errors;
	prev_rec_tiles[prev_idx] = recovered_tiles;
	prev_idx++;
	if (prev_idx >= WINDOW_SIZE) {
		prev_idx = 0;
	}

	if (window_cnt >= WINDOW_SIZE) {
		float avg_errors = 0, avg_tiles = 0;
		for (int i = 0; i < WINDOW_SIZE; i++) {
			avg_errors += prev_rec_errors[i];
			avg_tiles += prev_rec_tiles[i];
		}
		avg_errors /= WINDOW_SIZE;
		avg_tiles /= WINDOW_SIZE;

		int matrix_idx = ERR_MATRIX_ROWS - 1;
		for (int i = 0; i < ERR_MATRIX_ROWS; i++) {
			if (err_matrix[i].tiles <= min_tiles_percent) {
				matrix_idx = i;
				break;
			}
		}
		float err_thresholds[JPWL_CODES];
		memcpy_s(err_thresholds, JPWL_CODES * sizeof(err_thresholds[0]),
			err_matrix[matrix_idx].thresholds, JPWL_CODES * sizeof(err_matrix[matrix_idx].thresholds[0]));
		if (matrix_idx > 0 && min_tiles_percent > err_matrix[ERR_MATRIX_ROWS - 1].tiles) {
			float linear_approx_k = (min_tiles_percent - err_matrix[matrix_idx].tiles) /
				(err_matrix[matrix_idx - 1].tiles - err_matrix[matrix_idx].tiles);
			for (int i = 0; i < JPWL_CODES; i++) {
				err_thresholds[i] += (err_matrix[matrix_idx - 1].thresholds[i] - err_thresholds[i]) * linear_approx_k;
			}
		}
		// This RS code selection based on detected errors in stream
		int guessed_code_idx = JPWL_CODES - 1;
		for (int i = 1; i < JPWL_CODES; i++) {
			if (err_thresholds[i] > avg_errors) {
				guessed_code_idx = i - 1;
				break;
			}
		}

		float max_tiles_percent = min_tiles_percent > .88f ? .99f : (min_tiles_percent + .1f);
		int selected_code_idx = jpwl_idx;
		// Now we need to check more valuable parameter - recovered tiles percentage
		if (avg_tiles < min_tiles_percent) {
			int step = max((int)((min_tiles_percent - avg_tiles) * JPWL_CODES), 1);
			selected_code_idx = min(jpwl_idx + step, JPWL_CODES - 1);
			selected_code_idx = max(guessed_code_idx, selected_code_idx);
			window_cnt = WINDOW_SIZE >> 1;
		}
		else if (avg_tiles > max_tiles_percent) {
			selected_code_idx = (jpwl_idx + guessed_code_idx) >> 1;
			window_cnt = WINDOW_SIZE >> 1;
		}

		params->wcoder_data = jpwl_codes[selected_code_idx];
	}
	else {
		window_cnt++;
	}
}

//void init_err_matrix(wchar_t const* filename) {
//	FILE* in;
//	if (_wfopen_s(&in, filename, L"rt, ccs=UTF-8"))
//		return;
//	FILE* out;
//	if (_wfopen_s(&out, L"..\\Backup\\init_err_matrix.tsv", L"wt, ccs=UTF-8"))
//		return;
//	if (!in || !out)
//		return;
//	int rs_prev_idx = 0;
//	float err_prev = 0, tiles_prev = 1.0f, min_tiles_prev = 1.0f;
//	for (int e = 0; e < ERR_MATRIX_ROWS; e++) {
//		while (getwc(in) != L'\n');
//		fwprintf(out, L"%.2f\n", err_matrix[e].tiles);
//		char idx_rdy = 0;
//		do {
//			int rs, rs_idx = 0;
//			float err, tiles, _;
//			fwscanf_s(in, L"%d\t%f\t%f\t%f\n", &rs, &err, &tiles, &_);
//			tiles *= .01f;
//			for (int i = 0; i < JPWL_CODES; i++) {
//				if (jpwl_codes[i] == rs) {
//					rs_idx = i;
//					break;
//				}
//			}
//			if (rs_prev_idx != rs_idx) {
//				rs_prev_idx = rs_idx;
//				tiles_prev = 1.0f;
//				err_prev = err > 1.0f ? err - 1.0f : 0;
//				idx_rdy = 0;
//			}
//			if (idx_rdy) continue;
//			if (tiles == err_matrix[e].tiles) {
//				err_matrix[e].thresholds[rs_idx] = err * .01f;
//				fwprintf(out, L"%.1f\t", err);
//				idx_rdy = 1;
//			}
//			else if (tiles_prev > err_matrix[e].tiles && err_matrix[e].tiles > tiles) {
//				float deltaT = tiles_prev - tiles;
//				float deltaE = err - err_prev;
//				float k = (tiles_prev - err_matrix[e].tiles) / deltaT;
//				err = err_prev + deltaE * k;
//				err_matrix[e].thresholds[rs_idx] = err * .01f;
//				fwprintf(out, L"%.1f\t", err);
//				idx_rdy = 1;
//			}
//
//			tiles_prev = tiles;
//			err_prev = err;
//		} while (!feof(in));
//		fwprintf(out, L"\n");
//		rewind(in);
//	}
//	fclose(in);
//	fclose(out);
//}
