using System.Collections.ObjectModel;

namespace WarehouseApp.ViewModel
{
    public class HistoryPageViewModel
    {
        public ObservableCollection<HistoryCardViewModel> Items { get; }

        public HistoryPageViewModel()
        {
            Items = new ObservableCollection<HistoryCardViewModel>();
        }

        // Асинхронный метод для загрузки элементов
        public async Task LoadItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?
                .Select(goods => new HistoryCardViewModel(goods)) ?? Enumerable.Empty<HistoryCardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }

        // Асинхронный метод для обновления элементов
        public async Task RefreshItemsAsync()
        {
            var appState = await AppState.GetInstanceAsync(); // Получаем инстанс AppState асинхронно

            var updatedItems = appState.Goods?
                .Select(goods => new HistoryCardViewModel(goods)) ?? Enumerable.Empty<HistoryCardViewModel>();

            Items.Clear();
            foreach (var item in updatedItems)
            {
                Items.Add(item);
            }
        }
    }
}
